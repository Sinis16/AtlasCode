# Minimum CMake version required
cmake_minimum_required(VERSION 3.13.1)
set(BOARD raytac_mdbt50_db_33/nrf52833)

# Find Zephyr package
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
FILE(GLOB app_sources src/*.c)


# Project name
project(cardSpi)

# Target board (MDBT50-DB-33 with nRF52833)


# Define macro for DS-TWR initiator
add_definitions(-DDS_TWR_INIT)

# Source files (relative to sinis/card/)
target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE ../cardRep/main.c)
target_sources(app PRIVATE ../cardRep/config_options.c)
target_sources(app PRIVATE ds_twr_initiator_sts.c)  # Assuming it’s in sinis/card/
target_sources(app PRIVATE ../cardRep/decadriver/deca_device.c)
target_sources(app PRIVATE ../cardRep/platform/port.c)
target_sources(app PRIVATE ../cardRep/platform/deca_sleep.c)
target_sources(app PRIVATE ../cardRep/platform/deca_spi.c)
target_sources(app PRIVATE ../cardRep/shared_data/shared_functions.c)

# Include directories (relative to sinis/card/)
target_include_directories(app PRIVATE ../cardRep/)
target_include_directories(app PRIVATE ../cardRep/decadriver/)
target_include_directories(app PRIVATE ../cardRep/platform/)
target_include_directories(app PRIVATE ../cardRep/compiler/)
target_include_directories(app PRIVATE ../cardRep/shared_data/)
