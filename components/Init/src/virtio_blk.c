/*
 * Copyright 2019, DornerWorks
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <autoconf.h>

#include <sel4platsupport/arch/io.h>
#include <sel4utils/vspace.h>
#include <sel4utils/iommu_dma.h>
#include <simple/simple_helpers.h>
#include <vka/capops.h>

#include <camkes.h>
#include <camkes/dataport.h>

#include <ethdrivers/virtio/virtio_pci.h>
#include <ethdrivers/virtio/virtio_net.h>
#include <ethdrivers/virtio/virtio_ring.h>
#include <satadrivers/virtio/virtio_blk.h>

#include "vmm/vmm.h"
#include "vmm/driver/pci_helper.h"
#include "vmm/driver/virtio_emul.h"
#include "vmm/platform/ioports.h"
#include "vmm/platform/guest_vspace.h"
#include "vmm/driver/camkes_mutex.h"

#include "vm.h"
#include "i8259.h"

#define VIRTIO_VENDOR_ID            0x1af4
#define VIRTIO_DEVICE_ID            0x1001

#define VIRTIO_BLK_IOBASE           0x8000
#define VIRTIO_QUEUE_SIZE           128
#define VIRTIO_BLK_DISK_BLK_SIZE    512
#define VIRTIO_BLK_IRQ              7
#define VIRTIO_BLK_SIZE_MAX         4096
#define VIRTIO_BLK_SEG_MAX          1


#define CACHE_LINE_SIZE             64
#define LATENCY_TIMER               64
#define SUBSYSTEM_ID                2

static int virtio_blk_emul_transfer(struct disk_driver *driver, uint8_t direction, uint64_t sector,
                                    uint32_t len, uintptr_t guest_buf_phys);
static void emul_raw_handle_irq(struct disk_driver *driver, int irq);
static void emul_raw_poll(struct disk_driver *driver);
static void emul_print_state(struct disk_driver* driver);
static void emul_low_level_init(struct disk_driver *driver, struct virtio_blk_config* cfg);

/* Structure for the virtio_blk interface. Contains the following:
 *   iobase - virtual iobase for the pci connection
 *   *emul - virtio_blk interface functions - defined in virtio_blk_emul.c
 *   *emul_driver - Structure for the disk driver interface
 *   ioops - IO Operation Function Structure
 */
typedef struct virtio_blk {
    unsigned int iobase;
    blkif_virtio_emul_t *emul;
    struct disk_driver *emul_driver;
    ps_io_ops_t ioops;
} virtio_blk_t;

/* Global pointer for the virtio_blk interface structure
 * Doesn't actually get used, but memory space is filled with proper values */
static virtio_blk_t *virtio_blk = NULL;

/* There are function typedefs and the structures declared in raw.h
 * The functions created above are linked to the structure for future use
 */
static raw_diskiface_funcs_t emul_driver_funcs =
{
    .raw_xfer = virtio_blk_emul_transfer,
    .raw_handleIRQ = emul_raw_handle_irq,
    .raw_poll = emul_raw_poll,
    .print_state = emul_print_state,
    .low_level_init = emul_low_level_init
};

int WEAK virtio_blk_mutex_lock(void)
{
    assert(!"should not be here");
    return 0;
}

int WEAK virtio_blk_mutex_unlock(void)
{
    assert(!"should not be here");
    return 0;
}

// mutex to protect shared event context
static camkes_mutex_t virtio_blk_mutex = (camkes_mutex_t) {
    .lock = virtio_blk_mutex_lock,
    .unlock = virtio_blk_mutex_unlock,
};

#define SATASERVER_STATUS_GOOD          0
#define SATASERVER_STATUS_NOT_DONE      1
#define SATASERVER_STATUS_INVALID_CONF  2
/*
 *    These functions have the WEAK declaration because of the CAmkES protocol
 *    requirements. Virtio_blk uses the Sataserver component with interface functions (declared as
 *    "sataserver") so the compiler needs these so the build doesn't fail; however, the "weak"
 *    attribute means they are overridden by the proper functions (tx, rx, get_capacity)
 */

volatile Buf *sataserver_iface_buf WEAK;

int WEAK sataserver_iface_tx(unsigned int sector, unsigned int len)
{
    assert(!"should not be here");
    return 0;
}

int WEAK sataserver_iface_rx(unsigned int sector, unsigned int len)
{
    assert(!"should not be here");
    return 0;
}

unsigned int WEAK sataserver_iface_get_capacity(void)
{
    assert(!"should not be here");
    return 0;
}

unsigned int WEAK sataserver_iface_get_status(void)
{
    assert(!"should not be here");
    return 0;
}

/*
 * Purpose: Use the PCI Bus to read
 *
 * Inputs:
 *   - *cookie: virtio blk emulation information
 *   - port_no: PCI port to use
 *   - size: number of bytes to read
 *   - *result: pointer to newly read data.
 *
 * Returns: 0 -> Success; -1 -> Failure
 *
 */
static int virtio_blk_io_in(void *cookie, unsigned int port_no, unsigned int size, unsigned int *result)
{
    virtio_blk_t *blk = (virtio_blk_t*)cookie;
    unsigned int offset = port_no - blk->iobase;
    return blk->emul->io_in(blk->emul, offset, size, result);
}

/*
 * Purpose: Use the PCI Bus to write
 *
 * Inputs:
 *   - *cookie: virtio blk emulation information
 *   - port_no: PCI port to use
 *   - size: number of bytes to write
 *   - value: what to write on the bus
 *
 * Returns: 0 -> Success; -1 -> Failure
 *
 */
static int virtio_blk_io_out(void *cookie, unsigned int port_no, unsigned int size, unsigned int value)
{
    int ret;
    virtio_blk_t *blk = (virtio_blk_t*)cookie;
    unsigned int offset = port_no - blk->iobase;
    ret = blk->emul->io_out(blk->emul, offset, size, value);
    return ret;
}

/*
 * Purpose: Determine the transfer direction (In/Out) and either read/write from the Dataport.
 *          If the transfer failed, set the return variable accordingly
 *
 * Inputs:
 *   - *driver: disk information
 *   - direction: read/write to the disk
 *   - sector: sector to manipulate (512 byte offset)
 *   - len: number of bytes
 *   - guest_buf_phys: physical buffer to share data with
 *
 * Returns: TX_COMPLETE or TX_FAILED based on the transfer status
 *
 */
static int virtio_blk_emul_transfer(struct disk_driver *driver, uint8_t direction, uint64_t sector,
                                    uint32_t len, uintptr_t guest_buf_phys)
{
    int status = 0;  /* Variable to ensure transfer succeeded */

    if(VIRTIO_BLK_T_IN == direction)
    {
        status = sataserver_iface_rx(sector, len);
        memcpy((void *)guest_buf_phys, (void *)sataserver_iface_buf, len);
    }
    else if(VIRTIO_BLK_T_OUT == direction)
    {
        memcpy((void *)sataserver_iface_buf, (void*)guest_buf_phys, len);
        status = sataserver_iface_tx(sector, len);
    }
    else if ((VIRTIO_BLK_T_SCSI_CMD == direction) ||
             (VIRTIO_BLK_T_FLUSH == direction) ||
             (VIRTIO_BLK_T_GET_ID == direction))
    {
        // Do nothing
    }
    else
    {
        printf("virtio_blk: Invalid Command (%d)\n", direction);
    }
    return (status != 0 ? VIRTIO_BLK_XFER_COMPLETE : VIRTIO_BLK_XFER_FAILED);
}

/*
 * Purpose: Handle an Interrupt (May not be necessary...)
 *
 * Inputs:
 *   - *driver: disk information
 *   - irq: interrupt to trigger
 *
 * Returns: void
 *
 */
static void emul_raw_handle_irq(struct disk_driver *driver, int irq)
{
    i8259_gen_irq(VIRTIO_BLK_IRQ);
}

/* NOT IMPLEMENTED */
static void emul_raw_poll(struct disk_driver *driver)
{
    assert(!"not implemented");
}

/* NOT IMPLEMENTED */
static void emul_print_state(struct disk_driver* driver)
{
    assert(!"not implemented");
}

/*
 * Purpose: Configure the virtio blk structure's capacity, max segments, and size.
 *
 * Inputs:
 *   - *driver: disk information
 *   - *cfg: virtio blk configuration structure pointer
 *
 * Returns: void
 *
 */
static void emul_low_level_init(struct disk_driver *driver, struct virtio_blk_config* cfg)
{
    cfg->capacity = sataserver_iface_get_capacity();
    cfg->seg_max = VIRTIO_BLK_SEG_MAX;
    cfg->size_max = VIRTIO_BLK_SIZE_MAX;
    cfg->blk_size = VIRTIO_BLK_DISK_BLK_SIZE;
}

/*
 * Purpose: Initialize the emulated functions so virtio_emul_blk.c can call all of this
 *
 * Inputs:
 *   - *driver: disk information
 *   - io_ops: IO operation structure
 *   - *config: virtio_blk configuration information
 *
 * Returns: 0
 *
 */
static int emul_driver_init(struct disk_driver *driver, ps_io_ops_t io_ops, void *config)
{
    virtio_blk_t *blk = (virtio_blk_t*)config;
    driver->disk_data = config;
    driver->dma_alignment = sizeof(uintptr_t);
    driver->i_fn = emul_driver_funcs;
    blk->emul_driver = driver;
    return 0;
}

/* DMA Functions for the IO Operations */
static void* malloc_dma_alloc(void *cookie, size_t size, int align, int cached, ps_mem_flags_t flags)
{
    assert(cached);
    int error;
    void *ret;
    error = posix_memalign(&ret, align, size);
    if (error) {
        return NULL;
    }
    return ret;
}

static void malloc_dma_free(void *cookie, void *addr, size_t size)
{
    free(addr);
}

static uintptr_t malloc_dma_pin(void *cookie, void *addr, size_t size)
{
    return (uintptr_t)addr;
}

static void malloc_dma_unpin(void *cookie, void *addr, size_t size) {
}

static void malloc_dma_cache_op(void *cookie, void *addr, size_t size, dma_cache_op_t op) {
}

/* Interrupt Function Handler for Virtual Machine Monitor - Doesn't really do anything since we're polling */
void virtio_blk_notify(vmm_t *vmm)
{
    // This is where handler code would go
}

/* Initialization Function for Virtio Blk */
void make_virtio_blk(vmm_t *vmm)
{
    unsigned int status = sataserver_iface_get_status();

    while(SATASERVER_STATUS_NOT_DONE == status)
    {
        status = sataserver_iface_get_status();
    }
    if(SATASERVER_STATUS_INVALID_CONF == status)
    {
        printf("Invalid Partition configuration\n");
        return;
    }

    /* Initialize the PCI Device Structure */
    vmm_pci_device_def_t *pci_config = malloc(sizeof(*pci_config));
    assert(pci_config);
    memset(pci_config, 0, sizeof(*pci_config));
    *pci_config = (vmm_pci_device_def_t) {
        .vendor_id = VIRTIO_VENDOR_ID,
        .device_id = VIRTIO_DEVICE_ID,
        .cache_line_size = CACHE_LINE_SIZE,
        .latency_timer = LATENCY_TIMER,
        .subsystem_id = SUBSYSTEM_ID,
        .interrupt_pin = VIRTIO_BLK_IRQ,
        .interrupt_line = VIRTIO_BLK_IRQ
    };

    /* Initialize the Virtual PCI Device's IO Read and Write Functions */
    vmm_pci_entry_t entry = (vmm_pci_entry_t) {
        .cookie = pci_config,
        .ioread = vmm_pci_mem_device_read,
        .iowrite = vmm_pci_entry_ignore_write
    };

    /* Set up the PCI's Base Address Registers (BAR) */
    vmm_pci_bar_t bars[1] = {{
            .ismem = 0,
            .address = VIRTIO_BLK_IOBASE,
            .size_bits = 6
        }};

    entry = vmm_pci_create_bar_emulation(entry, 1, bars);
    vmm_pci_add_entry(&vmm->pci, entry, NULL);
    virtio_blk_t *blk = malloc(sizeof(*blk));
    virtio_blk = blk;
    assert(blk);
    memset(blk, 0, sizeof(*blk));
    blk->iobase = VIRTIO_BLK_IOBASE;
    vmm_io_port_add_handler(&vmm->io_port, VIRTIO_BLK_IOBASE, VIRTIO_BLK_IOBASE + MASK(6), blk,
                            virtio_blk_io_in, virtio_blk_io_out, "VIRTIO PCI BLK");

    /* Initializes the IO Operations */
    ps_io_ops_t ioops;
    ioops.dma_manager = (ps_dma_man_t) {
        .cookie = NULL,
        .dma_alloc_fn = malloc_dma_alloc,
        .dma_free_fn = malloc_dma_free,
        .dma_pin_fn = malloc_dma_pin,
        .dma_unpin_fn = malloc_dma_unpin,
        .dma_cache_op_fn = malloc_dma_cache_op
    };

    /*  Gives any and all neccessary function information to the global structure */
    blk->emul = blkif_virtio_emul_init(ioops, VIRTIO_QUEUE_SIZE, &vmm->guest_mem.vspace,
                                       &virtio_blk_mutex, emul_driver_init, blk);
    assert(blk->emul);
}
