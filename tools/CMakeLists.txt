cmake_minimum_required(VERSION 3.2.2)
project(PHOBOS-TOOLS CXX)

add_definitions("-DASIO_STANDALONE")
include_directories(../external/asio/asio/include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -Wall -Wextra")

add_executable(seriallog seriallog.cc)
