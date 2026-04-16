cmake_minimum_required(VERSION 3.20)

project(SGCoastManeuversRTMaps LANGUAGES CXX)

#==============================================================================
# RTMAPS MODULE
#==============================================================================
set(MODULE_NAME rtmaps)
set(MODULE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

message(STATUS "RTMAPS_SDKDIR: ${RTMAPS_SDKDIR}")

#==============================================================================
# RTMAPS SDK DETECTION
#==============================================================================
if(NOT DEFINED RTMAPS_SDKDIR)
    if(DEFINED ENV{RTMAPS_SDKDIR})
        set(RTMAPS_SDKDIR "$ENV{RTMAPS_SDKDIR}" CACHE PATH "Path to the RTMaps installation directory")
    else()
        message(STATUS "RTMAPS_SDKDIR not defined. RTMaps components will be skipped.")
        return()
    endif()
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#==============================================================================
# OUTPUT DIRECTORIES
#==============================================================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/rtmaps)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/rtmaps)

#==============================================================================
# COMPILER SETTINGS
#==============================================================================
if(MSVC)
    add_compile_options(
        /wd4251
        /wd4996
        /wd4244
        /wd4267
        /wd4127
        /wd4100
        /wd4189
        /WX-
    )
endif()

#==============================================================================
# RTMAPS INTEGRATION
#==============================================================================
include("${RTMAPS_SDKDIR}/templates.u/rtmaps.cmake")

#==============================================================================
# RTMAPS PACKAGES
#==============================================================================
add_subdirectory(packages/SGCoastManeuvers.u)
