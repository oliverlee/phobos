option(PHOBOS_BUILD_PROJECT_DRUNLO "Build Drunlo (static simulator) demo" TRUE)
if(PHOBOS_BUILD_PROJECT_DRUNLO)
    add_subdirectory(drunlo)
endif()
