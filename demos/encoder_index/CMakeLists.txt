include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_chibios_executable(encoder-index
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    ${PHOBOS_COMMON_SRC}
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc)
