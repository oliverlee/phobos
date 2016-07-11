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
set(CHIBIOS_USE_PROCESS_STACKSIZE "0x3000"
    CACHE STRING "Stack size used by the main() thread." FORCE)

set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake/modules
    ${PROJECT_SOURCE_DIR}/external/bicycle/cmake/modules)
message("Using CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
set(BICYCLE_SOURCE_DIR ${PROJECT_SOURCE_DIR}/external/bicycle/src)
set(BICYCLE_SOURCE
    ${BICYCLE_SOURCE_DIR}/bicycle.cc
    ${BICYCLE_SOURCE_DIR}/parameters.cc)
set(BICYCLE_INCLUDE_DIR ${BICYCLE_SOURCE_DIR} ${BICYCLE_SOURCE_DIR}/../inc)
add_definitions("-DBICYCLE_USE_DOUBLE_PRECISION_REAL=FALSE")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated") # suppress deprecated warnings from Eigen

include_directories(inc)
include_directories(src) # definitions for template classes are placed in src directory

add_subdirectory(src)
add_subdirectory(demos)
add_subdirectory(projects)
