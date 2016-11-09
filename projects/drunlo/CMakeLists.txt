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

add_phobos_executable(drunlo
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    ${PHOBOS_SOURCE_DIR}/src/analog.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc
    ${PHOBOS_SOURCE_DIR}/src/packet/framing.cc
    ${PHOBOS_SOURCE_DIR}/src/packet/serialize.cc
    ${PROTO_SRCS} ${PROTO_HDRS}
    ${BICYCLE_SOURCE})
