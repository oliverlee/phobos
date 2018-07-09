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

option(PBPRINT_WITH_GZIP
    "Build pbprint with gzip support. Requires a system installation of Boost." TRUE)

if (PBPRINT_WITH_GZIP)
    find_package(Boost 1.67.0 REQUIRED COMPONENTS system filesystem iostreams)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    ../projects/proto/pose.proto
    ../projects/proto/simulation.proto)

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

if (PBPRINT_WITH_GZIP)
    target_link_libraries(pbprint Boost::filesystem Boost::iostreams)
    target_compile_definitions(pbprint PRIVATE PBPRINT_USE_BOOST)
endif()
