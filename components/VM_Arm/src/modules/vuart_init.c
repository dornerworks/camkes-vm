/*
 * Copyright 2019, DornerWorks
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <vmlinux.h>
#include <camkes.h>

#include <sel4vmmplatsupport/plat/devices.h>

extern WEAK void *serial_getchar_buf;
extern WEAK seL4_Word serial_getchar_notification_badge();

//this matches the size of the buffer in serial server
#define BUFSIZE 4088

static int handle_serial_console(vm_t *vmm, void *cookie UNUSED)
{
    struct {
        uint32_t head;
        uint32_t tail;
        char buf[BUFSIZE];
    } volatile *buffer = serial_getchar_buf;

    char c;
    while (buffer->head != buffer->tail) {
        c = buffer->buf[buffer->head];
        buffer->head = (buffer->head + 1) % sizeof(buffer->buf);
        vuart_handle_irq(c);
    }
}

/* Install vuart */
static void vconsole_init_module(vm_t *vm, void *cookie)
{
    if (serial_getchar_notification_badge) {
        int err = register_async_event_handler(serial_getchar_notification_badge(), handle_serial_console, NULL);
        ZF_LOGF_IF(err, "Failed to register_async_event_handler for make_virtio_con.");
        vm_install_vconsole(vm, guest_putchar_putchar);
    } else {
        ZF_LOGF("Failed to initialize vconsole. Set serial_getchar_attributes");
    }
}

DEFINE_MODULE(vconsole, NULL, vconsole_init_module)
