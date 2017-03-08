cmake_minimum_required(VERSION 3.2)
project(PHOBOS_TESTS)

enable_testing()
include_directories(../external/googletest)
include_directories(../external/googletest/include)
add_subdirectory(../external/googletest
    ${CMAKE_CURRENT_BINARY_DIR}/googletest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -Wall -Wextra")
# suppress warnings from googletest
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wno-missing-field-initializers -Wno-undef")

add_executable(test_cobs
  test_cobs.cc
  test_cobs_util.cc
  ../src/cobs.cc
)
target_include_directories(test_cobs PRIVATE ../inc)
target_link_libraries(test_cobs gtest_main)
add_test(NAME test_cobs COMMAND test_cobs)

add_executable(test_cobs_random
  test_cobs_random.cc
  test_cobs_util.cc
  ../src/cobs.cc
)
target_include_directories(test_cobs_random PRIVATE ../inc)
target_link_libraries(test_cobs_random gtest_main)
add_test(NAME test_cobs_random COMMAND test_cobs_random)
