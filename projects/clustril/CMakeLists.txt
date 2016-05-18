set(BOOST_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
find_host_package(Eigen3 3.2.4 REQUIRED)
find_host_package(Boost 1.58 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${BICYCLE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})

# Any request from Eigen to allocate memory from heap results in an assertion failure
add_definitions("-DEIGEN_NO_MALLOC")

# Disable narrowing warning ('int' to 'uint32_t') as the original datatype for
# input to ADC_SQR1_NUM_CH() macro is 'uint16_t' and is cast to 'int'.
set_source_files_properties(${PROJECT_SOURCE_DIR}/src/analog.cc
    PROPERTIES COMPILE_FLAGS -Wno-narrowing)

# The required stacksize is not the exact amount but large enough such that the
# main thread does not overflow.
set(PROJECT_CLUSTRIL_REQUIRED_STACKSIZE "0x3000")
if(${CHIBIOS_USE_PROCESS_STACKSIZE} LESS ${PROJECT_CLUSTRIL_REQUIRED_STACKSIZE})
    message(SEND_ERROR "\
Increase process stacksize to ${PROJECT_CLUSTRIL_REQUIRED_STACKSIZE} \
prevent stack overflow in this project.")
endif()

add_chibios_executable(clustril
    chconf.h halconf.h mcuconf.h
    main.cc ${PHOBOS_SRC} ${BICYCLE_SOURCE})
target_compile_options(clustril PUBLIC "-Wno-undef") # suppress boost undef warnings
