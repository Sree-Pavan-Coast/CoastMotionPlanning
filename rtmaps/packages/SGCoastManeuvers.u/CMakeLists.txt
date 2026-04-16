set(PCK "SGCoastManeuvers")
set(RTMAPS_PCK "${PCK}.u" LANGUAGES CXX)
set(RTMAPS_PCK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

if(NOT DEFINED RTMAPS_SDKDIR)
    if(DEFINED ENV{RTMAPS_SDKDIR})
        set(RTMAPS_SDKDIR "$ENV{RTMAPS_SDKDIR}" CACHE PATH "Path to the RTMaps installation directory")
    else()
        message(FATAL_ERROR "RTMAPS_SDKDIR not defined. Either pass -DRTMAPS_SDKDIR=... to CMake or define an RTMAPS_SDKDIR environment variable")
    endif()
endif()

add_rtmaps_package(${PCK} PCKINFO "${PCK}.pckinfo"
    #coast path planer
    "src/maps_sgcoastpathplanner.cpp"
    "include/maps_sgcoastpathplanner.h"

    #pathtoprovisional
    "src/maps_sgpathtoprovisional.cpp"
    "include/maps_sgpathtoprovisional.h"
)

target_include_directories(${PCK} PRIVATE
    ${RTMAPS_PCK_INCLUDE_DIR}
    ${coast_motion_planning_SOURCE_DIR}/include
    $<BUILD_INTERFACE:${RTMAPS_PCK_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${RTMAPS_PCK_INCLUDE_DIR}>
    
)

target_link_libraries(${PCK} rtmaps_pck
    coast_motion_planning
    Boost::boost
)

if(MSVC)
    target_compile_definitions(${PCK} PRIVATE _ITERATOR_DEBUG_LEVEL=0)
    target_compile_options(${PCK} PRIVATE /W4)
else()
    target_compile_options(${PCK} PRIVATE -Wall -Wextra -pedantic)
endif()

set_target_properties(${PCK} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/rtmaps"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/rtmaps"
    FOLDER "RTMaps"
)
