include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# exclude printf source and default USB config
set(PHOBOS_COMMON_SRC
    ${PROJECT_BINARY_DIR}/src/gitsha1.cc
    ${PROJECT_SOURCE_DIR}/src/blink.cc
    ${PROJECT_SOURCE_DIR}/src/usbcfg.c)

# change optimization level to reduce used flash size
set(CMAKE_C_FLAGS_DEBUG "-gdwarf-4 -fvar-tracking-assignments -Og")
set(CMAKE_CXX_FLAGS_DEBUG "-gdwarf-4 -fvar-tracking-assignments -Og")

generate_protobuf()

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
    add_phobos_executable(flimnap_kinematic ${FLIMNAP_COMMON_SRC}
        ${PHOBOS_PROJECT_SOURCE_DIR}/haptic.cc)
    target_compile_definitions(flimnap_kinematic
        PRIVATE USE_BICYCLE_KINEMATIC_MODEL)
endif()
