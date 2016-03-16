include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_chibios_executable(gpt-adc
    chconf.h halconf.h mcuconf.h usbcfg.h
    main.cc usbcfg.c
    )
