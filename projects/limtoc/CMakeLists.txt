include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(LIMTOC_COMMON_SRC
    chconf.h
    halconf.h
    mcuconf.h
    main.cc
    ${PHOBOS_SOURCE_DIR}/src/analog.cc
    ${PHOBOS_SOURCE_DIR}/src/encoder.cc
    ${PHOBOS_SOURCE_DIR}/src/extconfig.cc)

#add_phobos_executable(limtoc_torque ${LIMTOC_COMMON_SRC})
add_phobos_executable(limtoc_velocity ${LIMTOC_COMMON_SRC})
target_compile_definitions(limtoc_velocity PRIVATE LIMTOC_VELOCITY_MODE)
