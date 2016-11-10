set(BOOST_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
find_host_package(Eigen3 3.2.4 REQUIRED)
find_host_package(Boost 1.58 REQUIRED)
find_host_package(Nanopb REQUIRED)

include_directories(${BICYCLE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${NANOPB_INCLUDE_DIRS})

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
set(PHOBOS_PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
set(PHOBOS_PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${PHOBOS_PROJECT_INCLUDE_DIR})

option(PHOBOS_BUILD_PROJECT_CLUSTRIL "Build Clustril (static simulator) demo" TRUE)
if(PHOBOS_BUILD_PROJECT_CLUSTRIL)
    add_subdirectory(clustril)
endif()

option(PHOBOS_BUILD_PROJECT_DRUNLO "Build Drunlo (static simulator) demo" TRUE)
if(PHOBOS_BUILD_PROJECT_DRUNLO)
    add_subdirectory(drunlo)
endif()

option(PHOBOS_BUILD_PROJECT_FLIMNAP "Build Flimnap (static simulator) demo" TRUE)
if(PHOBOS_BUILD_PROJECT_FLIMNAP)
    add_subdirectory(flimnap)
endif()
