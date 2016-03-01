project(PHOBOS)
cmake_minimum_required(VERSION 3.0)

# set ChibiOS board file
set(CHIBIOS_BOARD_CMAKE_FILE
    external/ChibiOS/os/hal/boards/OLIMEX_STM32_H405/board.cmake)
include(external/ChibiOS/chibios.cmake)

set(OPENOCD_INTERFACE_CFG ${PROJECT_SOURCE_DIR}/openocd_olimex-arm-usb-tiny-h_stm32f4.cfg)

add_subdirectory(demos)
