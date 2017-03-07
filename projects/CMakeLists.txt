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

# define protobuf generation macro
macro(generate_protobuf_source)
    set(argn ${ARGN})
    list(LENGTH argn n)
    if(${n} GREATER 0)
        find_host_package(Nanopb REQUIRED)
        include_directories(${NANOPB_INCLUDE_DIRS})

        nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS ${ARGN})
        include_directories(${CMAKE_CURRENT_BINARY_DIR})

        set(PROTOBUF_COMPILE_DEFINITIONS)
        foreach(proto in ${ARGN})
            get_filename_component(proto_name ${proto} NAME)
            if(proto_name MATCHES "simulation.proto")
                set(PROTOBUF_COMPILE_DEFINITIONS "PB_FIELD_16BIT")
                break()
            endif()
        endforeach()

        if(CMAKE_BUILD_TYPE MATCHES Release)
            set(PROTOBUF_COMPILE_DEFINITIONS ${PROTOBUF_COMPILE_DEFINITIONS}
                "PB_NO_ERRMSG" "PB_BUFFER_ONLY")
        endif()
        set_property(SOURCE ${PROTO_SRCS} APPEND PROPERTY COMPILE_DEFINITIONS
            ${PROTOBUF_COMPILE_DEFINITIONS})
        set(PROTOBUF_GENERATED_SOURCE ${PROTO_SRCS} ${PROTO_HDRS})
    endif()
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
if(PHOBOS_BUILD_PROJECT_FLIMNAP_WHIPPLE OR PHOBOS_BUILD_PROJECT_FLIMNAP_KINEMATIC)
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
