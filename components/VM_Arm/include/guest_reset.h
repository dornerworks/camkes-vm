/*
 * Copyright 2022, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(DATA61_GPL)
 */

#pragma once

#include <sel4vm/guest_vm.h>

/**
 * @function reset_virtqueue_virtio_net(vm)
 * Reset eth driver cookie and virtio_net
 * @param {vm_t *} vm           A handle to the VM that ram needs the virtqueue reset
 */
void reset_virtqueue_virtio_net(vm_t *vm);

/**
 * @function reset_ram(vm)
 * Mark all regions as un-allocated, collapse them, and set the ram region to 0
 * @param {vm_t *} vm           A handle to the VM that ram needs to be reset
 */
void reset_ram(vm_t *vm);
