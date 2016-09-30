include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Override default C/CXX debug optimization flag due to linking errors when both
# LTO and Debug mode are used. As stated in the gcc manual, the last -O option
# is used when there are multiple.
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Og")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og")

add_chibios_executable(dac
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    ${PHOBOS_COMMON_SRC})
