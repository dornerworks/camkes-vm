/*
 * Copyright 2017, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(DATA61_GPL)
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <autoconf.h>
#include <netinet/ether.h>

#include <platsupport/delay.h>
#include <sel4platsupport/arch/io.h>
#include <sel4utils/vspace.h>
#include <sel4utils/iommu_dma.h>
#include <simple/simple_helpers.h>
#include <vka/capops.h>
#include <utils/util.h>

#include <camkes.h>
#include <camkes/dataport.h>
#include <camkes/virtqueue.h>

#include <vswitch.h>

#include <vmlinux.h>
#include <virtio/virtio_net.h>

#include "virtio_net_interface.h"

#define MAX_BUFFER_ATTEMPTS 10
#define BUFFER_DELAY_US     10000 /* 10 ms */

static vswitch_t g_vswitch;
static struct ether_addr g_my_mac;
static virtio_net_t *virtio_net = NULL;

static void virtio_net_notify_vswitch_send(vswitch_node_t *node)
{
    volatile void *used_buff = NULL;
    size_t used_buff_sz = 0;

    int dequeue_res = virtqueue_driver_dequeue(node->virtqueues.send_queue, &used_buff, &used_buff_sz);
    if (dequeue_res) {
        ZF_LOGE("Unable to dequeue used buff");
        return;
    }

    camkes_virtqueue_buffer_free(node->virtqueues.send_queue, used_buff);
}

static int tx_virtqueue_forward(char *eth_buffer, size_t length, vswitch_node_t *destnode)
{
    volatile void *alloc_buffer = NULL;
    virtqueue_driver_t *const send_virtqueue = destnode->virtqueues.send_queue;

    int err = camkes_virtqueue_buffer_alloc(send_virtqueue, &alloc_buffer, length);
    if (err) {
        int i = 0;
        /* Wait til previous buffer is cleared */
        while (i < MAX_BUFFER_ATTEMPTS) {
            err = virtqueue_driver_poll(send_virtqueue);
            if (err) {
                virtio_net_notify_vswitch_send(destnode);
                break;
            }
            ps_udelay(BUFFER_DELAY_US);
            i++;
        }

        /* Alloc a new buffer */
        err = camkes_virtqueue_buffer_alloc(send_virtqueue, &alloc_buffer, length);
        if (err) {
            ZF_LOGE("Unable to allocate virtqueue buffer");
            return -1;
        }
    }

    memcpy((void *)alloc_buffer, (void *)eth_buffer, length);

    err = virtqueue_driver_enqueue(send_virtqueue, alloc_buffer, length);
    if (err) {
        ZF_LOGE("Unknown error while enqueuing available buffer (err = %d)", err);
        camkes_virtqueue_buffer_free(send_virtqueue, alloc_buffer);
        return -1;
    }

    err = virtqueue_driver_signal(send_virtqueue);
    ZF_LOGE_IF(err, "Unknown error while signaling sending virtqueue (err = %d)", err);

    return err;
}

static int virtio_net_vswitch_forward(char *eth_buffer, size_t length, virtio_net_t *virtio_net)
{
    struct ether_addr *destaddr;
    int err, destnode_start_idx, destnode_n_idxs;

    /* Initialize a convenience pointer to the dest macaddr.
     * The dest MAC addr is the first member of an ethernet frame.
     */
    destaddr = (struct ether_addr *)eth_buffer;

    /* Set up the bounds of the loop below that copies the frames into the destination Guest's virtqueue. */
    if (mac802_addr_eq_bcast(destaddr)) {
        /* Send to all nodes on the VLAN if destaddr is bcast addr. */
        destnode_n_idxs = g_vswitch.n_connected;
        destnode_start_idx = 0;
    } else if (mac802_addr_eq_ipv6_mcast(destaddr)) {
        // Do nothing and return
        return 0;
    } else {
        /* Send only to the target node */
        destnode_n_idxs = 1;
        destnode_start_idx = vswitch_get_destnode_index_by_macaddr(&g_vswitch, destaddr);
        if (destnode_start_idx < 0) {
            ZF_LOGW("Unreachable dest macaddr " PR_MAC802_ADDR ". Dropping frame.", PR_MAC802_ADDR_ARGS(destaddr));
            return -1;
        }
    }

    /* Copy the frame into the virtqueue of each of the targets we decided upon. */
    for (int i = destnode_start_idx; i < (destnode_start_idx + destnode_n_idxs); i++) {
        vswitch_node_t *destnode;

        destnode = vswitch_get_destnode_by_index(&g_vswitch, i);
        if (destnode == NULL) {
            /* This could happen in the broadcast case if there are holes in the array,
             * though that would still be odd. */
            ZF_LOGW("Found holes in node array while sending to dest MAC " PR_MAC802_ADDR,
                    PR_MAC802_ADDR_ARGS(destaddr));
            continue;
        }

        err = tx_virtqueue_forward(eth_buffer, length, destnode);
        ZF_LOGW_IF(err, "Unknown error while forwarding packet for dest " PR_MAC802_ADDR,
                   PR_MAC802_ADDR_ARGS(destaddr));

        /* This is a minor optimisation. We yield here to give the other end a chance to consume our transmitted packet.
         * When we resume at this point we then have the chance to send further packets, avoiding the need to drop them.
         */
        seL4_Yield();
        if (virtqueue_driver_poll(destnode->virtqueues.send_queue) == 1) {
            virtio_net_notify_vswitch_send(destnode);
        }
    }

    ZF_LOGD("Routed packet to " PR_MAC802_ADDR, PR_MAC802_ADDR_ARGS(destaddr));
    return 0;
}

/*
 * We recieve packets from other VM's through using our virtqueue
 * implementation. The current virtqueue implementation is limited to
 * recieving a single packet at a time. Our RX implementation is
 * limited by the following:
 * - Can't recieve payloads larger than 4K (maximum size of buffer)
 * - Can recive only one packet at once
 *   - We yield between packet consumes to give the other end a
 *   chance to send further virtqueue buffers.
 */
static void virtio_net_notify_vswitch_recv(vswitch_node_t *node)
{
    int err;
    virtqueue_t *rxdata_buff;
    ssize_t rxdata_buff_sz;

    if (node == NULL) {
        ZF_LOGE("Please connect this Guest (" PR_MAC802_ADDR ") to the VSwitch before trying to use the library.",
                PR_MAC802_ADDR_ARGS(&g_my_mac));
        return;
    }

    /* The eth frame is already in the virtqueue. It was placed there by the sender's libvswitch instance.
     * Check the virtqueue and pull it out.
     */
    volatile void *available_buff = NULL;
    size_t available_buff_sz = 0;
    int dequeue_res = virtqueue_device_dequeue(node->virtqueues.recv_queue, &available_buff, &available_buff_sz);

    while (dequeue_res >= 0) {
        void *cookie, *emul_buf;
        size_t len = available_buff_sz;
        int enqueue_res = 0;
        /* Allocate ring space to put the eth frame into. */
        emul_buf = (void *)virtio_net->emul_driver->i_cb.allocate_rx_buf(virtio_net->emul_driver->cb_cookie,
                                                                         available_buff_sz, &cookie);

        if (emul_buf == NULL) {
            ZF_LOGW("Dropping frame for " PR_MAC802_ADDR ": No ring mem avail.", PR_MAC802_ADDR_ARGS(&g_my_mac));

            enqueue_res = virtqueue_device_enqueue(node->virtqueues.recv_queue, available_buff, available_buff_sz);
            ZF_LOGE_IF(enqueue_res, "Unable to enqueue frame at " PR_MAC802_ADDR, PR_MAC802_ADDR_ARGS(&g_my_mac));

            break;
        }

        memcpy(emul_buf, (void *)available_buff, len);

        virtio_net->emul_driver->i_cb.rx_complete(virtio_net->emul_driver->cb_cookie, 1, &cookie, (unsigned int *)&len);

        enqueue_res = virtqueue_device_enqueue(node->virtqueues.recv_queue, available_buff, available_buff_sz);
        ZF_LOGE_IF(enqueue_res, "Unable to enqueue frame at " PR_MAC802_ADDR, PR_MAC802_ADDR_ARGS(&g_my_mac));

        /* We yield here to give the other end a chance to send more virtqueues for us to consume. This is a minor
         * optimisation to avoid having to re-enter the event loop on our global notification object. */
        seL4_Yield();
        dequeue_res = virtqueue_device_dequeue(node->virtqueues.recv_queue, &available_buff, &available_buff_sz);
    }
    err = virtqueue_device_signal(node->virtqueues.recv_queue);
    ZF_LOGW_IF(err, "Failed to signal on virtqueue meant for Guest " PR_MAC802_ADDR, PR_MAC802_ADDR_ARGS(&g_my_mac));
}

void virtio_net_notify(vm_t *vm UNUSED)
{
    for (int i = 0; i < VSWITCH_NUM_NODES; i++) {
        vswitch_node_t *node = &g_vswitch.nodes[i];
        if (node->virtqueues.send_queue != NULL) {
            if (virtqueue_driver_poll(node->virtqueues.send_queue) == 1) {
                virtio_net_notify_vswitch_send(node);
            }
        }
        if (node->virtqueues.recv_queue != NULL) {
            if (virtqueue_device_poll(node->virtqueues.recv_queue) == 1) {
                virtio_net_notify_vswitch_recv(node);
            }
        }
    }
}

static void get_self_mac_addr(struct ether_addr *self_addr)
{
    struct ether_addr *res = ether_aton_r(vswitch_mac_address, self_addr);
    ZF_LOGF_IF(NULL == res, "Failed to get MAC address");
}

static int make_vswitch_net(void)
{
    int err = 0;
    err = vswitch_init(&g_vswitch);
    if (err) {
        ZF_LOGE("Unable to initialise vswitch library");
        return err;
    }

    get_self_mac_addr(&g_my_mac);

    int num_vswitch_entries = ARRAY_SIZE(vswitch_layout);
    for (int i = 0; i < num_vswitch_entries; i++) {
        struct vswitch_mapping mac_mapping = vswitch_layout[i];
        virtqueue_driver_t *send_virtqueue;
        virtqueue_device_t *recv_virtqueue;
        int local_err = camkes_virtqueue_driver_init(&send_virtqueue, mac_mapping.send_id);
        if (local_err) {
            ZF_LOGE("Unable to initialise send virtqueue for %s", mac_mapping.mac_addr);
            err = local_err;
            continue;
        }
        local_err = camkes_virtqueue_device_init(&recv_virtqueue, mac_mapping.recv_id);
        if (local_err) {
            ZF_LOGE("Unable to initialise recv virtqueue for %s", mac_mapping.mac_addr);
            camkes_virtqueue_driver_free(send_virtqueue);
            err = local_err;
            continue;
        }
        struct ether_addr guest_macaddr;
        struct ether_addr *res = ether_aton_r(mac_mapping.mac_addr, &guest_macaddr);
        ZF_LOGF_IF(res == NULL, "Failed to get MAC address");

        local_err = vswitch_connect(&g_vswitch, &guest_macaddr, send_virtqueue, recv_virtqueue);
        if (local_err) {
            ZF_LOGE("Unable to initialise vswitch for MAC address: %s", mac_mapping.mac_addr);
            camkes_virtqueue_driver_free(send_virtqueue);
            camkes_virtqueue_device_free(recv_virtqueue);
            err = local_err;
            continue;
        }
        ZF_LOGD("Initialized vswitch connection: send_id = %d; recv_id = %d; mac_addr = %s",
                mac_mapping.send_id, mac_mapping.recv_id, mac_mapping.mac_addr);
    }
    return err;
}

void make_virtio_net_vswitch(vm_t *vm, void *cookie)
{
    int rv = make_vswitch_net();
    ZF_LOGE_IF(rv, "make_vswitch_net() reported non-fatal error (%d)", rv);

    virtio_net_callbacks_t callbacks;
    callbacks.tx_callback = virtio_net_vswitch_forward;
    callbacks.irq_callback = NULL;
    virtio_net = virtio_net_init(vm, &callbacks);
    ZF_LOGF_IF(NULL == virtio_net, "Failed to initialize virtio_net_init()");
}

DEFINE_MODULE(virtio_net_vswitch, NULL, make_virtio_net_vswitch)
