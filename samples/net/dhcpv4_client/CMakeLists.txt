# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/sonance_zephyr_bsp)

set(BOARD hummingbird_evk/mimxrt1176/cm7)

#../../../../../ptp/sonance_zephyr_bsp/
project(dhcpv4_client)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
