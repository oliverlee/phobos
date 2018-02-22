set(BOOST_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
set(USE_BICYCLE_EIGEN3_SUBMODULE TRUE)
set(USE_BICYCLE_BOOST_SUBMODULE TRUE)

include_directories(${BICYCLE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})

# Allow printing of float types
add_definitions("-DCHPRINTF_USE_FLOAT")
# use static simulator configurations
add_definitions("-DSTATIC_SIMULATOR_CONFIG")

# The required stacksize is not the exact amount but large enough such that the
# main thread does not overflow.
set(PROJECT__REQUIRED_STACKSIZE "0x3000")
if(${CHIBIOS_USE_PROCESS_STACKSIZE} LESS ${PROJECT__REQUIRED_STACKSIZE})
    message(SEND_ERROR "\
Increase process stacksize to ${PROJECT__REQUIRED_STACKSIZE} \
prevent stack overflow in this project.")
endif()

# include project headers and sources
set(PHOBOS_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PHOBOS_PROJECT_PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto)
set(PHOBOS_PROJECT_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PHOBOS_PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${PHOBOS_PROJECT_INCLUDE_DIR})

# define protobuf generation macro as dependencies for add_custom_command only
# works with targets created in the same directory
macro(generate_protobuf)
    find_host_package(PythonInterp REQUIRED)
    add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/txrx.proto
        COMMAND ${PYTHON_EXECUTABLE} ${PHOBOS_PROJECT_PROTO_DIR}/generate_proto.py
            ${PHOBOS_PROJECT_PROTO_DIR}/txrx.proto.in ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS ${PHOBOS_PROJECT_PROTO_DIR}/generate_proto.py
            ${PHOBOS_PROJECT_PROTO_DIR}/txrx.proto.in
            ${PHOBOS_PROJECT_PROTO_DIR}/txrx.options
        COMMENT "Generating proto schema ${CMAKE_CURRENT_BINARY_DIR}/txrx.proto")

    set(PROTO_SCHEMA
        ${PHOBOS_PROJECT_PROTO_DIR}/build.proto
        ${PHOBOS_PROJECT_PROTO_DIR}/simulation.proto
        ${PHOBOS_PROJECT_PROTO_DIR}/pose.proto
        ${CMAKE_CURRENT_BINARY_DIR}/txrx.proto)

    find_host_package(Nanopb REQUIRED)
    include_directories(${NANOPB_INCLUDE_DIRS})

    foreach(proto ${PROTO_SCHEMA})
        set(NANOPB_DEPENDS)
        set(NANOPB_IMPORT_DIRS)
        if(CMAKE_BUILD_TYPE MATCHES Release)
            set(PROTOBUF_COMPILE_DEFINITIONS "PB_NO_ERRMSG" "PB_BUFFER_ONLY")
        endif()

        get_filename_component(proto_name ${proto} NAME)
        if(proto_name MATCHES "txrx.proto")
            # manually set dependencies
            set(NANOPB_DEPENDS
                ${PHOBOS_PROJECT_PROTO_DIR}/build.options
                ${PHOBOS_PROJECT_PROTO_DIR}/simulation.options)
            set(NANOPB_IMPORT_DIRS
                ${PHOBOS_PROJECT_PROTO_DIR})
            set(PROTOBUF_COMPILE_DEFINITIONS
                ${PROTOBUF_COMPILE_DEFINITIONS} "PB_FIELD_16BIT")
        endif()

        nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS ${proto})
        set_property(SOURCE ${PROTO_SRCS}
            APPEND PROPERTY COMPILE_DEFINITIONS ${PROTOBUF_COMPILE_DEFINITIONS})
    endforeach()

    set(PROTOBUF_GENERATED_SOURCE ${PROTO_SRCS} ${PROTO_HDRS})
    list(REMOVE_DUPLICATES PROTOBUF_GENERATED_SOURCE)
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
endmacro()

option(PHOBOS_BUILD_PROJECT_CLUSTRIL "Build Clustril (static simulator) demo" TRUE)
if(PHOBOS_BUILD_PROJECT_CLUSTRIL)
    add_subdirectory(clustril)
endif()

option(PHOBOS_BUILD_PROJECT_DRUNLO "Build Drunlo (static simulator) demo" TRUE)
if(PHOBOS_BUILD_PROJECT_DRUNLO)
    add_subdirectory(drunlo)
endif()

option(PHOBOS_BUILD_PROJECT_FLIMNAP_WHIPPLE
    "Build Flimnap (static simulator) demo using the Whipple bicycle model" TRUE)
option(PHOBOS_BUILD_PROJECT_FLIMNAP_KINEMATIC
    "Build Flimnap (static simulator) demo using the kinematic bicycle model" TRUE)
if(PHOBOS_BUILD_PROJECT_FLIMNAP_WHIPPLE OR
   PHOBOS_BUILD_PROJECT_FLIMNAP_KINEMATIC)
    add_subdirectory(flimnap)
endif()

option(PHOBOS_BUILD_PROJECT_GULLIVER "Build Gulliver (static simulator) demo" TRUE)
if(PHOBOS_BUILD_PROJECT_GULLIVER)
    add_subdirectory(gulliver)
endif()

option(PHOBOS_BUILD_PROJECT_HALL "Build Hall (static simulator) demo" TRUE)
if(PHOBOS_BUILD_PROJECT_HALL)
    add_subdirectory(hall)
endif()

option(PHOBOS_BUILD_PROJECT_LIMTOC "Build Limtoc (static simulator) demo" TRUE)
if(PHOBOS_BUILD_PROJECT_LIMTOC)
    add_subdirectory(limtoc)
endif()

option(PHOBOS_BUILD_PROJECT_RELDRESAL "Build Reldresal (static simulator) demo" TRUE)
if(PHOBOS_BUILD_PROJECT_RELDRESAL)
    add_subdirectory(reldresal)
endif()
