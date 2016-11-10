include_directories(${CMAKE_CURRENT_SOURCE_DIR})

generate_protobuf_source(${PHOBOS_PROJECT_PROTO_DIR}/pose.proto)

# suppress Boost undef warnings and Eigen deprecated warnings
set_property(SOURCE
    ${PHOBOS_PROJECT_SOURCE_DIR}/virtualbicycle.cc
    APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-undef -Wno-deprecated")

add_phobos_executable(flimnap
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/virtualbicycle.cc
    ${PHOBOS_SOURCE_DIR}/src/analog.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc
    ${PROTOBUF_GENERATED_SOURCE}
    ${BICYCLE_SOURCE})
