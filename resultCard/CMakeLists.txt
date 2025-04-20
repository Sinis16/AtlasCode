# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(BOARD raytac_mdbt50_db_33/nrf52833)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})



include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
FILE(GLOB src/*.c)

project(result)

set(BOARD raytac_mdbt50_db_33/nrf52833)

add_definitions(-DDS_TWR_INIT)

target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE ../cardNewRep/main.c)
target_sources(app PRIVATE ../cardNewRep/config_options.c)
target_sources(app PRIVATE src/ds_twr_initiator_sts.c) 
target_sources(app PRIVATE ../cardNewRep/decadriver/deca_device.c)
target_sources(app PRIVATE ../cardNewRep/platform/port.c)
target_sources(app PRIVATE ../cardNewRep/platform/deca_sleep.c)
target_sources(app PRIVATE ../cardNewRep/platform/deca_spi.c)
target_sources(app PRIVATE ../cardNewRep/shared_data/shared_functions.c)

target_include_directories(app PRIVATE ../cardNewRep/)
target_include_directories(app PRIVATE ../cardNewRep/decadriver/)
target_include_directories(app PRIVATE ../cardNewRep/platform/)
target_include_directories(app PRIVATE ../cardNewRep/compiler/)
target_include_directories(app PRIVATE ../cardNewRep/shared_data/)
