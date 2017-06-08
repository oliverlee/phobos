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

# change optimization level to reduce used flash size
set(CMAKE_C_FLAGS_DEBUG "-gdwarf-4 -fvar-tracking-assignments -Og")
set(CMAKE_CXX_FLAGS_DEBUG "-gdwarf-4 -fvar-tracking-assignments -Og")

# Always use release flags for bicycle sources
set_property(SOURCE ${BICYCLE_SOURCE}
    APPEND_STRING PROPERTY COMPILE_FLAGS " ${CMAKE_CXX_FLAGS_RELEASE}")

set(FLIMNAP_COMMON_SRC
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    serialize.cc
    usbconfig.c # add USB config file without input/output buffer queues
    ${PHOBOS_PROJECT_SOURCE_DIR}/haptic.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/messageutil.cc
    ${PHOBOS_PROJECT_SOURCE_DIR}/transmitter.cc
    ${PHOBOS_SOURCE_DIR}/src/analog.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc
    ${PHOBOS_SOURCE_DIR}/src/cobs.cc
    ${PROTOBUF_GENERATED_SOURCE}
    ${BICYCLE_SOURCE})

# suppress warnings that protobuf fields do not match original declarations when using LTO
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-lto-type-mismatch")
# suppress warnings from ChibiOS serial USB
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-maybe-uninitialized")

if(PHOBOS_BUILD_PROJECT_FLIMNAP_WHIPPLE)
add_phobos_executable(flimnap_whipple ${FLIMNAP_COMMON_SRC})
endif()
if(PHOBOS_BUILD_PROJECT_FLIMNAP_KINEMATIC)
add_phobos_executable(flimnap_kinematic ${FLIMNAP_COMMON_SRC})
target_compile_definitions(flimnap_kinematic
    PRIVATE USE_BICYCLE_KINEMATIC_MODEL)
endif()
if(PHOBOS_BUILD_PROJECT_FLIMNAP_ZERO_INPUT)
add_phobos_executable(flimnap_zero_input ${FLIMNAP_COMMON_SRC})
target_compile_definitions(flimnap_zero_input
    PRIVATE FLIMNAP_ZERO_INPUT)
endif()
