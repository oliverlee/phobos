set(BOOST_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
find_host_package(Eigen3 3.2.4 REQUIRED)
find_host_package(Boost 1.58 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${BICYCLE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})

# Any request from Eigen to allocate memory from heap results in an assertion failure
add_definitions("-DEIGEN_NO_MALLOC")

add_chibios_executable(bicycle
    chconf.h halconf.h mcuconf.h
    main.cc ${BICYCLE_SOURCE})
target_compile_options(bicycle PUBLIC "-Wno-undef") # suppress boost undef warnings
