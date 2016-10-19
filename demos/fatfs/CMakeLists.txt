include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# use ChibiOS FATFS module
set(CHIBIOS_BUILD_WITH_FATFS ON)

add_phobos_executable(fatfs
    chconf.h
    ffconf.h
    halconf.h
    mcuconf.h
    main.cc)
