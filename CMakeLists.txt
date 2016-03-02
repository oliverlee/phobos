project(PHOBOS)
cmake_minimum_required(VERSION 3.0)

# set ChibiOS board file and OpenOCD config file
set(CHIBIOS_BOARD_CMAKE_FILE "${PROJECT_SOURCE_DIR}/board/board.cmake"
    CACHE FILEPATH "ChibiOS Board CMake file.")
set(OPENOCD_INTERFACE_CFG "${PROJECT_SOURCE_DIR}/openocd_olimex-arm-usb-tiny-h_stm32f4.cfg"
    CACHE FILEPATH "OpenOCD interface configuration file.")

# include ChibiOS CMake script
include(external/ChibiOS/chibios.cmake)

add_subdirectory(demos)
