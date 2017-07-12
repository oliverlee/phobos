include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_phobos_executable(hall
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    ${PHOBOS_SOURCE_DIR}/src/analog.cc)
