include_directories(${CMAKE_CURRENT_SOURCE_DIR})

generate_protobuf_source(${PHOBOS_PROJECT_PROTO_DIR}/simulation.proto
                         ${PHOBOS_PROJECT_PROTO_DIR}/pose.proto)

# exclude printf source and default USB config
set(PHOBOS_COMMON_SRC
    ${PROJECT_BINARY_DIR}/src/gitsha1.cc
    ${PROJECT_SOURCE_DIR}/src/blink.cc)

# suppress Boost undef warnings and Eigen deprecated warnings
set_property(SOURCE
    ${PHOBOS_PROJECT_SOURCE_DIR}/haptic.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/messageutil.cc
    APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-undef -Wno-deprecated")

add_phobos_executable(flimnap
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    serialize.cc
    usbconfig.c # add USB config file without input/output buffer queues
    ${PHOBOS_PROJECT_SOURCE_DIR}/haptic.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/messageutil.cc
    ${PHOBOS_SOURCE_DIR}/src/analog.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc
    ${PHOBOS_SOURCE_DIR}/src/packet/frame.cc
    ${PROTOBUF_GENERATED_SOURCE}
    ${BICYCLE_SOURCE})
