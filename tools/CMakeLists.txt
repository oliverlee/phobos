cmake_minimum_required(VERSION 3.2.2)
project(PHOBOS-TOOLS CXX C)

if ($ENV{TRAVIS})
    # Use protobuf install directory on Travis
    set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/protobuf)
endif()
if (CMAKE_COMPILER_IS_GNUCXX)
    # ignore protobuf unused parameters
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
endif()
find_package(Protobuf REQUIRED)

add_definitions("-DASIO_STANDALONE")
include_directories(../external/asio/asio/include)
include_directories(../inc)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
# suppress asio warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

set(PHOBOS_PROJECT_PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../projects/proto)

find_package(PythonInterp REQUIRED)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/txrx.proto
    COMMAND ${PYTHON_EXECUTABLE} ${PHOBOS_PROJECT_PROTO_DIR}/generate_proto.py
        ${PHOBOS_PROJECT_PROTO_DIR}/txrx.proto.in ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${PHOBOS_PROJECT_PROTO_DIR}/generate_proto.py
        ${PHOBOS_PROJECT_PROTO_DIR}/txrx.proto.in
        ${PHOBOS_PROJECT_PROTO_DIR}/txrx.options
    COMMENT "Generating proto schema ${CMAKE_CURRENT_BINARY_DIR}/txrx.proto")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    ${PHOBOS_PROJECT_PROTO_DIR}/build.proto
    ${PHOBOS_PROJECT_PROTO_DIR}/pose.proto
    ${PHOBOS_PROJECT_PROTO_DIR}/simulation.proto
    ${CMAKE_CURRENT_BINARY_DIR}/txrx.proto)

add_executable(seriallog seriallog.cc)
add_executable(pbprint pbprint.cc ../src/cobs.cc ${PROTO_SRCS} ${PROTO_HDRS})
# enable warnings for unused parameters for source files
set_property(SOURCE seriallog.cc pbprint.cc ../src/cobs.cc
    APPEND_STRING PROPERTY COMPILE_FLAGS " -Wunused-parameter")
target_link_libraries(pbprint ${PROTOBUF_LIBRARIES})
if (NOT APPLE)
    find_package(Threads)
    target_link_libraries(seriallog ${CMAKE_THREAD_LIBS_INIT})
    target_link_libraries(pbprint ${CMAKE_THREAD_LIBS_INIT})
endif()
