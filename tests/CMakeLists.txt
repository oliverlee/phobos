cmake_minimum_required(VERSION 3.2)

project(phobos_tests)

add_executable(test_cobs
  test_cobs.cc
  ../src/cobs.cc
)

target_include_directories(test_cobs PRIVATE ../inc)
