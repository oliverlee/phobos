include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_phobos_executable(adc
    chconf.h
    halconf.h
    mcuconf.h
    main.cc)
