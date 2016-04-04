include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_chibios_executable(usb-serial
    chconf.h halconf.h mcuconf.h usbcfg.h
    main.c usbcfg.c)
