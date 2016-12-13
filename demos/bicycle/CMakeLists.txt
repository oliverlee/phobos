set(BOOST_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
set(USE_BICYCLE_EIGEN3_SUBMODULE TRUE)
find_host_package(Boost 1.58 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${BICYCLE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})

# Allow printing of float types
add_definitions("-DCHPRINTF_USE_FLOAT")

# The required stacksize is not the exact amount but large enough such that the
# main thread does not overflow.
set(DEMO_BICYCLE_REQUIRED_STACKSIZE "0x3000")
if(${CHIBIOS_USE_PROCESS_STACKSIZE} LESS ${DEMO_BICYCLE_REQUIRED_STACKSIZE})
    message(SEND_ERROR "\
Increase process stacksize to ${DEMO_BICYCLE_REQUIRED_STACKSIZE} \
prevent stack overflow in this project.")
endif()

add_phobos_executable(bicycle
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    ${BICYCLE_SOURCE})
