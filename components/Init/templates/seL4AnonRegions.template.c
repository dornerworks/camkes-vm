/*
 * Copyright 2024, DornerWorks
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <assert.h>
#include <camkes/error.h>
#include <stdio.h>
#include <stdint.h>
#include <sel4/sel4.h>

typedef struct region {
    uintptr_t addr;
    size_t size;
} region_t;

/*- set default_regions = [{ "addr": 0x10003000, "size": 0x8FFFD000 }] -*/
/*- set regions = configuration[me.name].get("anonymous_regions", default_regions) -*/
static region_t anonymous_regions[] = {
/*- for region in regions -*/
    {.addr = /*? region['addr'] ?*/, .size = /*? region['size'] ?*/},
/*- endfor -*/
};

int anonymous_num_regions(void) {
    return ARRAY_SIZE(anonymous_regions);
}

void anonymous_get_region(int region_num, uintptr_t *addr, size_t *size) {
    assert(addr);
    assert(size);
    assert(region_num < anonymous_num_regions());
    *addr = anonymous_regions[region_num].addr;
    *size = anonymous_regions[region_num].size;
}
