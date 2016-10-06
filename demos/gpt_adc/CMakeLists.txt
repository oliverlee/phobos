include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_phobos_executable(gpt-adc
    chconf.h
    halconf.h
    mcuconf.h
    main.cc)
