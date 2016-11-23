include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(CHIBIOS_USE_LTO AND (CMAKE_BUILD_TYPE MATCHES Release))
    add_definitions("-DLTO_SYSCALL_UNDEFINED_REFERNCE_WORKAROUND")
endif()

# Allow printing of float types
add_definitions("-DCHPRINTF_USE_FLOAT")

add_phobos_executable(encoder
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc)
