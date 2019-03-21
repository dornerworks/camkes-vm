#
# Copyright 2018, Data61
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230.
#
# Copyright 2019, DornerWorks
#
# This software may be distributed and modified according to the terms of
# the BSD 2-Clause license. Note that NO WARRANTY is provided.
# See "LICENSE_BSD2.txt" for details.
#
# @TAG(DATA61_DORNERWORKS_BSD)
#

cmake_minimum_required(VERSION 3.8.2)

project(Sataserver C)

DeclareCAmkESComponent(Sataserver
    SOURCES src/sataserver.c
    INCLUDES include
    LIBS sel4allocman sel4platsupport satadrivers
    )

if(LibSatadriversAHCIEnable)
    CAmkESAddCPPInclude("${CMAKE_CURRENT_LIST_DIR}/include/plat/${KernelPlatform}/${CAmkESVMDestHardware}")
endif()
