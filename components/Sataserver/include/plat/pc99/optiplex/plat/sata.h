/*
 * Copyright 2019, DornerWorks
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(DORNERWORKS_GPL)
 */

#pragma once

#define HARDWARE_SATA_PROVIDES_INTERFACES                                          \
    dataport Buf(8192) mmio;
    /**/

#define HARDWARE_SATA_INTERFACES                                                   \
    dataport Buf(8192) ahcidriver;
    /**/

#define HARDWARE_SATA_COMPOSITION                                                  \
    component HWSata HWsata;                                                       \
    connection seL4HardwareMMIO satadrivermmio(from ahcidriver, to HWsata.mmio);
    /**/

#define HARDWARE_SATA_CONFIG                                                       \
    /* In AHCI mode the PCI device has an associated memory space */               \
    HWsata.mmio_paddr = 0xff870000;                                                \
    HWsata.mmio_size = 0x2000;
    /**/
