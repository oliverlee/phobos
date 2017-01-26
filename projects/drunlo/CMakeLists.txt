include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(CHIBIOS_USE_LTO AND
        ((CMAKE_BUILD_TYPE MATCHES Release) OR (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)))
    add_definitions("-DLTO_SYSCALL_UNDEFINED_REFERNCE_WORKAROUND")
endif()

add_phobos_executable(drunlo
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    ${PHOBOS_SOURCE_DIR}/src/analog.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc)
