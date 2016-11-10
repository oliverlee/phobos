include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# use ChibiOS FATFS module
set(CHIBIOS_BUILD_WITH_FATFS ON)

# generate protobuf sources
nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS ${PHOBOS_PROJECT_DIR}/proto/clustril.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
set(PROTOBUF_COMPILE_DEFINITIONS "PB_FIELD_16BIT")
if(CMAKE_BUILD_TYPE MATCHES Release)
    set(PROTOBUF_COMPILE_DEFINITIONS ${PROTOBUF_COMPILE_DEFINITIONS} "PB_NO_ERRMSG" "PB_BUFFER_ONLY")
endif()
set_property(SOURCE ${PROTO_SRCS} APPEND PROPERTY COMPILE_DEFINITIONS
    ${PROTOBUF_COMPILE_DEFINITIONS})

# exclude blink and usb sources
set(PHOBOS_COMMON_SRC ${PROJECT_BINARY_DIR}/src/gitsha1.cc)

# suppress Boost undef warnings and Eigen deprecated warnings
set_property(SOURCE
    ${PHOBOS_PROJECT_SOURCE_DIR}/messageutil.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/virtualbicycle.cc
    APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-undef -Wno-deprecated")

add_phobos_executable(clustril
    chconf.h
    ffconf.h
    halconf.h
    mcuconf.h
    main.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/messageutil.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/filesystem.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/virtualbicycle.cc
    ${PHOBOS_SOURCE_DIR}/src/analog.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc
    ${PHOBOS_SOURCE_DIR}/src/packet/frame.cc
    ${PHOBOS_SOURCE_DIR}/src/packet/serialize.cc
    ${PROTO_SRCS} ${PROTO_HDRS}
    ${BICYCLE_SOURCE})
