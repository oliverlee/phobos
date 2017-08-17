include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# use ChibiOS FATFS module
set(CHIBIOS_BUILD_WITH_FATFS ON)

generate_protobuf_source(
    ${PHOBOS_PROJECT_PROTO_DIR}/config.proto
    ${PHOBOS_PROJECT_PROTO_DIR}/pose.proto
    ${PHOBOS_PROJECT_PROTO_DIR}/simulation.proto)

# exclude blink and usb sources
set(PHOBOS_COMMON_SRC ${PROJECT_BINARY_DIR}/src/gitsha1.cc)

# suppress Boost undef warnings and Eigen deprecated warnings
set_property(SOURCE
    ${PHOBOS_PROJECT_SOURCE_DIR}/haptic.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/messageutil.cc
    APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-undef -Wno-deprecated")

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
    ${PHOBOS_SOURCE_DIR}/src/analog.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc
    ${PROTOBUF_GENERATED_SOURCE}
    ${BICYCLE_SOURCE})
