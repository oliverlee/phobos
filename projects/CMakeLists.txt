option(PHOBOS_BUILD_PROJECT_CLUSTRIL "Build Clustril (static simulator) demo" TRUE)
if(PHOBOS_BUILD_PROJECT_CLUSTRIL)
    add_subdirectory(clustril)
endif()
