#
# Copyright 2018, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.8.2)

set(configure_string "")
if("${CAmkESVMDestHardware}" STREQUAL "optiplex")
    list(APPEND configure_string "#define CAMKES_VM_DEST_HW_OPTIPLEX")
elseif("${CAmkESVMDestHardware}" STREQUAL "klaw")
    list(APPEND configure_string "#define CAMKES_VM_DEST_HW_KLAW")
elseif("${CAmkESVMDestHardware}" STREQUAL "qemu")
    list(APPEND configure_string "#define CAMKES_VM_DEST_HW_QEMU")
else()
    message(FATAL_ERROR "No hardware platform set in camkes-vm")
endif()

if(LibSel4VMMUseHPET)
    list(APPEND configure_string "#define CONFIG_VMM_USE_HPET")
endif()


config_option(
    CAmkESVMGuestDMAIommu
    CAMKES_VM_GUEST_DMA_IOMMU
    "Enable DMA through the IOMMU"
    DEFAULT
    ON
)

# Create VMM configuration library
add_config_library(camkes_vmm "${configure_string}")

# Common VM build definitions
CAmkESAddImportPath(components)
CAmkESAddImportPath(interfaces)
CAmkESAddTemplatesPath(templates)
CAmkESAddTemplatesPath(components/Init/templates)

# Add VM Components and Libraries
add_subdirectory(components/Echo)
add_subdirectory(components/Firewall)
add_subdirectory(components/StringReverse)
add_subdirectory(components/UDPServer)
add_subdirectory(components/Sataserver)

# Declare connectors with templates.
# We can rely on the naming scheme being consistent to allow for easier declaration.
foreach(
    connector
    IN
    ITEMS
    seL4UDPSend
    seL4UDPRecv
    seL4PicoUDPSend
    seL4PicoUDPRecv
    seL4MultiSharedData
    seL4Sataserver
)
    DeclareCAmkESConnector(
        ${connector}
        FROM
        ${connector}-from.template.c
        TO
        ${connector}-to.template.c
    )
endforeach()
