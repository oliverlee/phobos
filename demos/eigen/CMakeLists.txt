find_host_package(Eigen3 3.2.4 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

# Allow printing of float types
add_definitions("-DCHPRINTF_USE_FLOAT")

add_phobos_executable(eigen
    chconf.h
    halconf.h
    mcuconf.h
    main.cc)
