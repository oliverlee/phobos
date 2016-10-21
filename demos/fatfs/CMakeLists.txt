include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(CHIBIOS_USE_LTO AND
        ((CMAKE_BUILD_TYPE MATCHES Release) OR (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)))
    add_definitions("-DLTO_SYSCALL_UNDEFINED_REFERNCE_WORKAROUND")
endif()

# use ChibiOS FATFS module
set(CHIBIOS_BUILD_WITH_FATFS ON)

add_phobos_executable(fatfs
    chconf.h
    ffconf.h
    halconf.h
    mcuconf.h
    main.cc)
