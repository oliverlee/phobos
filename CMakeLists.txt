project(PHOBOS)
cmake_minimum_required(VERSION 3.0)

# set ChibiOS board file and OpenOCD config file
set(CHIBIOS_BOARD_CMAKE_FILE "${PROJECT_SOURCE_DIR}/board/board.cmake"
    CACHE FILEPATH "ChibiOS Board CMake file.")
set(OPENOCD_INTERFACE_CFG "${PROJECT_SOURCE_DIR}/openocd_olimex-arm-usb-tiny-h_stm32f4.cfg"
    CACHE FILEPATH "OpenOCD interface configuration file.")

# include ChibiOS CMake script
# WARNING: this script clears all compile and link flags that have been set
include(external/ChibiOS/chibios.cmake)
set(CHIBIOS_USE_PROCESS_STACKSIZE "0x3000"
    CACHE STRING "Stack size used by the main() thread.")
if(CMAKE_BUILD_TYPE MATCHES Release)
    add_definitions("-DOSAL_DBG_ENABLE_ASSERTS=false -DOSAL_DBG_ENABLE_CHECKS=false")
else()
    add_definitions("-DOSAL_DBG_ENABLE_ASSERTS=true -DOSAL_DBG_ENABLE_CHECKS=true")
endif()

# define CMake module path and include external project CMake modules
set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake/modules
    ${PROJECT_SOURCE_DIR}/external/bicycle/cmake/modules
    ${PROJECT_SOURCE_DIR}/external/nanopb/extra)
message(STATUS "Using CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

# bicycle setup
set(BICYCLE_SOURCE_DIR ${PROJECT_SOURCE_DIR}/external/bicycle/src)
set(BICYCLE_SOURCE
    ${BICYCLE_SOURCE_DIR}/bicycle/bicycle.cc
    ${BICYCLE_SOURCE_DIR}/bicycle/bicycle_solve_constraint_pitch.cc
    ${BICYCLE_SOURCE_DIR}/bicycle/kinematic.cc
    ${BICYCLE_SOURCE_DIR}/bicycle/whipple.cc
    ${BICYCLE_SOURCE_DIR}/parameters.cc)
set(BICYCLE_INCLUDE_DIR ${BICYCLE_SOURCE_DIR} ${BICYCLE_SOURCE_DIR}/../inc)
add_definitions("-DBICYCLE_USE_DOUBLE_PRECISION_REAL=false")
set_property(SOURCE ${BICYCLE_SOURCE} APPEND_STRING PROPERTY COMPILE_FLAGS
    "-fno-math-errno -fassociative-math -freciprocal-math -fcx-limited-range")

# use nested submodules in bicycle submodule
set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/bicycle/external/eigen)
include(${PROJECT_SOURCE_DIR}/external/bicycle/cmake/boost.cmake) # set Boost_INCLUDE_DIR
#set(Boost_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/bicycle/external/boost)

# define nanopb variables if later needed
set(NANOPB_SRC_ROOT_FOLDER ${PROJECT_SOURCE_DIR}/external/nanopb)

# get git sha1 https://stackoverflow.com/questions/1435953/how-can-i-pass-git-sha1-to-compiler-as-definition-using-cmake
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GITSHA1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z")

## Define macro for phobos project executable
# This macro adds common sources for all targets in this project and
# conditionally defines compile flags to disable specific warnings related to
# external libraries and definitions for those external libraries.
macro(add_phobos_executable target_name)
    add_chibios_executable(${target_name} ${ARGN} ${PHOBOS_COMMON_SRC})
    set(source_file_properties_modified 0)
    foreach(src in ${ARGN})
        get_filename_component(src_name ${src} NAME)
        if(src_name MATCHES "main.cc")
            # Disable format warnings for printf as we use parameters types defined by ChibiOS
            set_property(SOURCE ${src} APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-format")
            # suppress Boost undef warnings
            if(USE_BICYCLE_BOOST_SUBMODULE)
                set_property(SOURCE ${src} APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-undef")
            endif()
            # suppress deprecated warnings from Eigen
            if(USE_BICYCLE_EIGEN3_SUBMODULE)
                set_property(SOURCE ${src} APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-deprecated")
            endif()
            # suppress ChibiOS usb serial warning
            set_property(SOURCE ${src} APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-maybe-uninitialized")
            math(EXPR source_file_properties_modified
                "${source_file_properties_modified} + 1")
        endif()
        # Disable narrowing warning ('int' to 'uint32_t') as the original datatype for
        # input to ADC_SQR1_NUM_CH() macro is 'uint16_t' and is cast to 'int'.
        if(src_name STREQUAL "analog.cc")
            set_property(SOURCE ${src} APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-narrowing")
            math(EXPR source_file_properties_modified
                "${source_file_properties_modified} + 1")
        endif()
        # Determine if bicycle sources are included in this project
        set(USE_BICYCLE_SOURCE FALSE)
        if(src_name STREQUAL "bicycle.cc")
            set(USE_BICYCLE_SOURCE TRUE)
            math(EXPR source_file_properties_modified
                "${source_file_properties_modified} + 1")
        endif()
        if(source_file_properties_modified GREATER 2)
            break()
        endif()
    endforeach()

    if(USE_BICYCLE_BOOST_SUBMODULE)
        # suppress boost undef warnings
        set_property(SOURCE ${BICYCLE_SOURCE} APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-undef")
        set_property(SOURCE ${ARGN} APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-misleading-indentation")
    endif()
    if(USE_BICYCLE_EIGEN3_SUBMODULE)
        # Any request from Eigen to allocate memory from heap results in an assertion failure
        add_definitions("-DEIGEN_NO_MALLOC")
        # suppress deprecated warnings from Eigen
        set_property(SOURCE ${BICYCLE_SOURCE} APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-deprecated")
    endif()
    if(USE_BICYCLE_SOURCE_)
        add_definitions("-DBICYCLE_USE_DOUBLE_PRECISION_REAL=false")
    endif()
endmacro(add_phobos_executable target_name)

include_directories(inc)
include_directories(src) # definitions for template classes are placed in src directory
add_subdirectory(src)

option(PHOBOS_BUILD_DEMOS "Build embedded demos" TRUE)
if(PHOBOS_BUILD_DEMOS)
    add_subdirectory(demos)
endif()

option(PHOBOS_BUILD_PROJECTS "Build embedded projects" TRUE)
if(PHOBOS_BUILD_PROJECTS)
    add_subdirectory(projects)
endif()

option(PHOBOS_BUILD_TOOLS "Build host machine tools" TRUE)
if(PHOBOS_BUILD_TOOLS)
    include(ExternalProject)
    ExternalProject_Add(phobos_tools
        PREFIX ${PROJECT_BINARY_DIR}
        TMP_DIR ""
        STAMP_DIR ""
        DOWNLOAD_DIR ""
        SOURCE_DIR ${PROJECT_SOURCE_DIR}/tools
        BINARY_DIR ${PROJECT_BINARY_DIR}/tools
        CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        INSTALL_COMMAND "")
    set_directory_properties(PROPERTY
        ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_BINARY_DIR}/tools)
endif()

option(PHOBOS_BUILD_TESTS "Build host machine tests" TRUE)
if(PHOBOS_BUILD_TESTS)
    include(ExternalProject)
    ExternalProject_Add(phobos_tests
        PREFIX ${PROJECT_BINARY_DIR}
        TMP_DIR ""
        STAMP_DIR ""
        DOWNLOAD_DIR ""
        SOURCE_DIR ${PROJECT_SOURCE_DIR}/tests
        BINARY_DIR ${PROJECT_BINARY_DIR}/tests
        CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        INSTALL_COMMAND "")
    set_directory_properties(PROPERTY
        ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_BINARY_DIR}/tests)
endif()
