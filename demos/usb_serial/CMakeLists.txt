include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_chibios_executable(ch
    chconf.h halconf.h mcuconf.h usbcfg.h
    main.c usbcfg.c)
