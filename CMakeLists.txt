project(PHOBOS)
cmake_minimum_required(VERSION 3.0)

# set ChibiOS board file and OpenOCD config file
set(CHIBIOS_BOARD_CMAKE_FILE "${PROJECT_SOURCE_DIR}/board/board.cmake"
    CACHE FILEPATH "ChibiOS Board CMake file.")
set(OPENOCD_INTERFACE_CFG "${PROJECT_SOURCE_DIR}/openocd_olimex-arm-usb-tiny-h_stm32f4.cfg"
    CACHE FILEPATH "OpenOCD interface configuration file.")

# include ChibiOS CMake script
# WARNING: this script clears all compile and link flags that have been set
include(external/ChibiOS/chibios.cmake)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
message("Using CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated") # suppress deprecated warnings from Eigen

add_subdirectory(demos)
