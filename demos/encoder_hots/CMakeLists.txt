set(USE_BICYCLE_EIGEN3_SUBMODULE TRUE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

# Allow printing of float types
add_definitions("-DCHPRINTF_USE_FLOAT")

add_phobos_executable(encoder-hots
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    #${PHOBOS_SOURCE_DIR}/src/encoder.cc # uses tsencoder.hh
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc)
