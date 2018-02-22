include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# use ChibiOS FATFS module
set(CHIBIOS_BUILD_WITH_FATFS ON)

# exclude blink and usb sources
set(PHOBOS_COMMON_SRC ${PROJECT_BINARY_DIR}/src/gitsha1.cc)

generate_protobuf()

add_phobos_executable(clustril
    chconf.h
    ffconf.h
    halconf.h
    mcuconf.h
    main.cc
    serialize.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/filesystem.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/haptic.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/messageutil.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc
    ${PROTOBUF_GENERATED_SOURCE}
    ${BICYCLE_SOURCE})
