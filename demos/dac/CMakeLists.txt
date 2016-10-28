include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(CHIBIOS_USE_LTO AND (CMAKE_BUILD_TYPE MATCHES Debug))
    add_definitions("-DLTO_SYSCALL_UNDEFINED_REFERNCE_WORKAROUND")
endif()

add_phobos_executable(dac
    chconf.h
    halconf.h
    mcuconf.h
    main.cc)
