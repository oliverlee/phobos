include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_host_package(PythonInterp REQUIRED)
add_custom_command(OUTPUT
    ${CMAKE_CURRENT_BINARY_DIR}/txrx.proto
    COMMAND ${PYTHON_EXECUTABLE} ${PHOBOS_PROJECT_PROTO_DIR}/generate_proto.py
        ${PHOBOS_PROJECT_PROTO_DIR}/txrx.proto.in ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${PHOBOS_PROJECT_PROTO_DIR}/generate_proto.py
        ${PHOBOS_PROJECT_PROTO_DIR}/txrx.proto.in
        ${PHOBOS_PROJECT_PROTO_DIR}/txrx.options
    COMMENT "Generating proto schema ${CMAKE_CURRENT_BINARY_DIR}/txrx.proto")

generate_protobuf_source(
    ${PHOBOS_PROJECT_PROTO_DIR}/build.proto
    ${PHOBOS_PROJECT_PROTO_DIR}/simulation.proto
    ${PHOBOS_PROJECT_PROTO_DIR}/pose.proto)
set(NANOPB_DEPENDS
    ${PHOBOS_PROJECT_PROTO_DIR}/build.options
    ${PHOBOS_PROJECT_PROTO_DIR}/simulation.options)
set(NANOPB_IMPORT_DIRS ${PHOBOS_PROJECT_PROTO_DIR})
generate_protobuf_source(
    ${CMAKE_CURRENT_BINARY_DIR}/txrx.proto)

# exclude printf source and default USB config
set(PHOBOS_COMMON_SRC
    ${PROJECT_BINARY_DIR}/src/gitsha1.cc
    ${PROJECT_SOURCE_DIR}/src/blink.cc
    ${PROJECT_SOURCE_DIR}/src/usbcfg.c)

# suppress Boost undef warnings and Eigen deprecated warnings
set_property(SOURCE
    ${PHOBOS_PROJECT_SOURCE_DIR}/messageutil.cc
    APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-undef -Wno-deprecated")

# change optimization level to reduce used flash size
set(CMAKE_C_FLAGS_DEBUG "-gdwarf-4 -fvar-tracking-assignments -Og")
set(CMAKE_CXX_FLAGS_DEBUG "-gdwarf-4 -fvar-tracking-assignments -Og")

set(FLIMNAP_COMMON_SRC
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    serialize.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/messageutil.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/receiver.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/transmitter.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/txrx_common.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc
    ${PHOBOS_SOURCE_DIR}/src/cobs.cc
    ${PROTOBUF_GENERATED_SOURCE}
    ${BICYCLE_SOURCE})

add_definitions("-DPROJECT_FLIMPNAP")

if(PHOBOS_BUILD_PROJECT_FLIMNAP_WHIPPLE)
add_phobos_executable(flimnap_whipple ${FLIMNAP_COMMON_SRC})
endif()
if(PHOBOS_BUILD_PROJECT_FLIMNAP_KINEMATIC)
add_phobos_executable(flimnap_kinematic ${FLIMNAP_COMMON_SRC})
target_compile_definitions(flimnap_kinematic
    PRIVATE USE_BICYCLE_KINEMATIC_MODEL)
endif()
