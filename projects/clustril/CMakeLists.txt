set(BOOST_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
find_host_package(Eigen3 3.2.4 REQUIRED)
find_host_package(Boost 1.58 REQUIRED)
find_host_package(Nanopb REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${BICYCLE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${NANOPB_INCLUDE_DIRS})

# Allow printing of float types
add_definitions("-DCHPRINTF_USE_FLOAT")
# use static simulator configurations
add_definitions("-DSTATIC_SIMULATOR_CONFIG")
# use ChibiOS FATFS module
set(CHIBIOS_BUILD_WITH_FATFS ON)

# The required stacksize is not the exact amount but large enough such that the
# main thread does not overflow.
set(PROJECT_CLUSTRIL_REQUIRED_STACKSIZE "0x3000")
if(${CHIBIOS_USE_PROCESS_STACKSIZE} LESS ${PROJECT_CLUSTRIL_REQUIRED_STACKSIZE})
    message(SEND_ERROR "\
Increase process stacksize to ${PROJECT_CLUSTRIL_REQUIRED_STACKSIZE} \
prevent stack overflow in this project.")
endif()

# generate protobuf sources
nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROJECT_SOURCE_DIR}/proto/messages.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
set(PROTOBUF_COMPILE_DEFINITIONS "PB_FIELD_16BIT")
if(CMAKE_BUILD_TYPE MATCHES Release)
    set(PROTOBUF_COMPILE_DEFINITIONS ${PROTOBUF_COMPILE_DEFINITIONS} "PB_NO_ERRMSG" "PB_BUFFER_ONLY")
endif()
set_property(SOURCE ${PROTO_SRCS} APPEND PROPERTY COMPILE_DEFINITIONS
    ${PROTOBUF_COMPILE_DEFINITIONS})

add_phobos_executable(clustril
    chconf.h
    ffconf.h
    halconf.h
    mcuconf.h
    main.cc
    printstate.cc
    virtualbicycle.cc
    ${PHOBOS_SOURCE_DIR}/src/analog.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc
    ${PHOBOS_SOURCE_DIR}/src/packet/framing.cc
    ${PHOBOS_SOURCE_DIR}/src/packet/serialize.cc
    ${PROTO_SRCS} ${PROTO_HDRS}
    ${BICYCLE_SOURCE})
