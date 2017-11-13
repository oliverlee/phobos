include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# exclude default USB config
set(PHOBOS_COMMON_SRC
    ${PROJECT_BINARY_DIR}/src/gitsha1.cc
    ${PROJECT_SOURCE_DIR}/src/blink.cc
    ${PROJECT_SOURCE_DIR}/src/printf.c)

add_phobos_executable(reldresal
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    usbconfig.c # add USB config file without input/output buffer queues
    ${PHOBOS_SOURCE_DIR}/src/analog.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc)
