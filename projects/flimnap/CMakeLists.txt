include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# generate protobuf sources
nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROJECT_SOURCE_DIR}/proto/messages.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
set(PROTOBUF_COMPILE_DEFINITIONS "PB_FIELD_16BIT")
if(CMAKE_BUILD_TYPE MATCHES Release)
    set(PROTOBUF_COMPILE_DEFINITIONS ${PROTOBUF_COMPILE_DEFINITIONS} "PB_NO_ERRMSG" "PB_BUFFER_ONLY")
endif()
set_property(SOURCE ${PROTO_SRCS} APPEND PROPERTY COMPILE_DEFINITIONS
    ${PROTOBUF_COMPILE_DEFINITIONS})

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
    ${PROTO_SRCS} ${PROTO_HDRS}
    ${BICYCLE_SOURCE})
