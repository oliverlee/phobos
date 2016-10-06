include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_phobos_executable(encoder-index
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc)
