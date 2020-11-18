/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

/**
 * @brief virtio console backend layer for ARM
 *
 * The virtio console backend has two layers, the emul layer and the backend layer.
 * This file is part of the backend layer, it's responsible for:
 *
 * - TX: receiving data from the emul layer and forwarding them to the destination
 *   port using camkes virtqueue (which is different from the virtqueue that's
 *   used to communicate between the guest and the emul layer).
 * - RX: receiving data from the source port via camkes virtqueue and invoking the
 *   emul layer to handle the data.
 * - virtio console backend initialization
 *
 * This file relies on the connection topology and camkes virtqueue connection
 * being configured correctly in camkes.
 *
 * @todo Handling the RX and TX is similar between the x86/ARM VMMs, hence this file
 * and `components/Init/src/virtio_con.c` share a lot of common code. A refactor for
 * the backend layer is needed.
 */

#include <stdint.h>
#include <string.h>
#include <autoconf.h>
#include <vmlinux.h>

#include <utils/util.h>

#include <camkes.h>
#include <camkes/dataport.h>

#include <sel4vmmplatsupport/drivers/virtio_con.h>
#include <sel4vmmplatsupport/device.h>
#include <sel4vmmplatsupport/arch/vpci.h>

#include <platsupport/serial.h>
#include <virtioarm/virtio_console.h>

#include <virtqueue.h>
#include <camkes/virtqueue.h>

/* Number of ports is the number of crossvm connections plus hvc0 */
#define NUM_PORTS (ARRAY_SIZE(serial_layout) + 1)

#include "serial_interface.h"

static virtio_con_t *virtio_con = NULL;
extern vmm_pci_space_t *pci;
extern vmm_io_port_list_t *io_ports;

/* 4088 because the serial_shmem_t struct has to be 0x1000 bytes big */
#define BUFSIZE (0x1000 - 2 * sizeof(uint32_t))

/**
 * This struct is a ring buffer that occupies exactly one page, it should represent
 * the data in the shmem region (that is shared between two nodes of a virtio console
 * connection.
 *
 * Example:
 *
 *    |----------*************------------|
 *    0        head         tail       BUFSIZE
 *
 *    (- available, * used)

 * Invariants of the ring buffer:
 * 1. 0 <= head < BUFSIZE, 0 <= tail < BUFSIZE
 * 2. used = (tail >= head) ? (tail - head) : (tail + BUFSIZE - head)
 * 3. empty = head - tail == 0
 * 4. full = (tail + 1) % BUFSIZE == head
 *
 * For this ring buffer, always add data to the tail and take from the head. The access
 * can be lock-free since an instance of this struct can only be either the receive buffer
 * or the sent buffer of a virtio console node, which guarantees that only one end of the
 * ring buffer struct can be modified in a node.
 */
typedef struct serial_shmem {
    uint32_t head;
    uint32_t tail;
    char buf[BUFSIZE];
} serial_shmem_t;
compile_time_assert(serial_shmem_4k_size, sizeof(serial_shmem_t) == 0x1000);

/*
 * Represents the virtqueues used in a given link between
 * two VMs.
 */
typedef struct serial_conn {
    void (*notify)(void);
    serial_shmem_t *recv_buf;
    serial_shmem_t *send_buf;
} serial_conn_t;
static serial_conn_t connections[NUM_PORTS];

/**
 * Writes data to the guest console attached to a port, and sets the head of the receive
 * ring buffer to the right place.
 *
 * @param port Port number
 */
static void virtio_con_notify_recv(int port)
{
    serial_shmem_t *buffer = connections[port].recv_buf;
    uint32_t last_read_tail = buffer->tail;

    virtio_con->emul_driver->emul_cb.emul_putchar(port, virtio_con->emul, buffer->buf, buffer->head, last_read_tail);
    buffer->head = last_read_tail;
}

/**
 * Callback function for camkes virtqueue notification. Polls each receive ring buffer
 * to see if there are data pending. For some legacy reasons, this callback passes a
 * VMM handler and a cookie as parameters, they're unused here.
 *
 * @see include/vmlinux.h (async_event_handler_fn_t) for usages
 *
 * @return Returns 0 on success.
 */
static int handle_serial_console(vm_t *vmm, void *cookie UNUSED)
{
    /* Poll each ring buffer to see if data was added */
    for (int i = 0; i < NUM_PORTS; i++) {
        if (connections[i].recv_buf->head != connections[i].recv_buf->tail) {
            virtio_con_notify_recv(i);
        }
    }
    return 0;
}

/**
 * Forwards data to the destination port using camkes virtqueue, and notifies the dest VM.
 *
 * @param port Port number
 * @param c Data to forward
 */
static void tx_virtcon_forward(int port, char c)
{
    if (port >= NUM_PORTS) {
        return;
    }

    serial_shmem_t *buffer = connections[port].send_buf;
    buffer->buf[buffer->tail] = c;
    __atomic_thread_fence(__ATOMIC_ACQ_REL);
    buffer->tail = (buffer->tail + 1) % BUFSIZE;

    /* If newline or staging area full, it's time to send */
    if (c == '\n' || (buffer->tail + 1) % BUFSIZE == buffer->head) {
        __atomic_thread_fence(__ATOMIC_ACQ_REL);
        connections[port].notify();
    }
}

/* camkes-generated symbols */
/* regions shared by the SerialServer and the VMM */
extern serial_shmem_t *batch_buf;
extern serial_shmem_t *serial_getchar_buf;

extern seL4_Word serial_getchar_notification_badge(void);

/**
 * Initialize a virtio console backend. This includes:
 * - adding an IO port handler
 * - adding a PCI entry
 * - initializing the emul layer
 * - setting up connections (includes hvc0)
 *
 * @see include/vmlinux.h (vmm_module_t) for usages
 *
 * @param vm The vm handler of the caller
 * @param cookie Unused in this function
 */
void make_virtio_con(vm_t *vm, void *cookie UNUSED)
{
    ZF_LOGF_IF((NUM_PORTS > VIRTIO_CON_MAX_PORTS), "Too many ports configured (up the constant)");

    virtio_con = virtio_console_init(vm, tx_virtcon_forward, pci, io_ports);
    ZF_LOGF_IF(!virtio_con, "Failed to initialise virtio console");

    int err;
    for (int i = 0; i < NUM_PORTS; i++) {
        if (i == 0) {
            /* Port 0 is the existing SerialServer shmem interface */
            connections[0].notify = batch_batch;
            connections[0].recv_buf = serial_getchar_buf;
            connections[0].send_buf = batch_buf;
            err = register_async_event_handler(serial_getchar_notification_badge(), handle_serial_console, NULL);
            if (err) {
                ZF_LOGE("Failed to register event handler");
                return;
            }
            continue;
        }

        camkes_virtqueue_channel_t *recv_channel = get_virtqueue_channel(VIRTQUEUE_DEVICE, serial_layout[i - 1].recv_id);
        camkes_virtqueue_channel_t *send_channel = get_virtqueue_channel(VIRTQUEUE_DRIVER, serial_layout[i - 1].send_id);

        if (!recv_channel || !send_channel) {
            ZF_LOGE("Failed to get channel");
            return;
        }

        err = register_async_event_handler(recv_channel->recv_badge, handle_serial_console, NULL);
        if (err) {
            ZF_LOGE("Failed to register event handler");
            return;
        }

        connections[i].notify = send_channel->notify;
        connections[i].recv_buf = (serial_shmem_t *) recv_channel->channel_buffer;
        connections[i].send_buf = (serial_shmem_t *) send_channel->channel_buffer;
    }
}

DEFINE_MODULE(virtio_con, NULL, make_virtio_con)
