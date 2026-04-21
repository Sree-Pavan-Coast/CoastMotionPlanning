cmake_minimum_required(VERSION 3.18)
project(coast_motion_planning VERSION 1.0.0 LANGUAGES CXX)

option(COAST_BUILD_RTMAPS "Build RTMAPS package integration" ON)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

include(FetchContent)

# Find Boost
find_package(Boost REQUIRED)
find_package(ompl CONFIG QUIET
    HINTS
        /opt/homebrew/opt/ompl/share/ompl/cmake
        /opt/homebrew/Cellar/ompl/1.7.0_1/share/ompl/cmake
)

if(NOT ompl_FOUND)
    FetchContent_Declare(
        ompl
        GIT_REPOSITORY https://github.com/ompl/ompl.git
        GIT_TAG        1.7.0
    )
    set(OMPL_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
    set(OMPL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(OMPL_BUILD_PYBINDINGS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(ompl)
endif()

if(TARGET ompl AND NOT TARGET ompl::ompl)
    add_library(ompl::ompl ALIAS ompl)
endif()

# Find yaml-cpp via FetchContent since it's missing on system
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        0.8.0
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)

# Fetch Eigen3 (Header-only mode to avoid uninstall target collision and test builds)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
)
FetchContent_GetProperties(eigen)
if(NOT TARGET Eigen3::Eigen AND NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()

# Fetch ANYbotics grid_map_core
FetchContent_Declare(
    grid_map_core
    GIT_REPOSITORY https://github.com/anybotics/grid_map.git
    GIT_TAG        master
)
FetchContent_GetProperties(grid_map_core)
if(NOT grid_map_core_POPULATED)
    FetchContent_Populate(grid_map_core)
    
    # Manually define grid_map_core to avoid catkin dependencies
    add_library(grid_map_core STATIC
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/GridMap.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/GridMapMath.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/SubmapGeometry.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/BufferRegion.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/Polygon.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/CubicInterpolation.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/iterators/GridMapIterator.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/iterators/SubmapIterator.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/iterators/CircleIterator.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/iterators/EllipseIterator.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/iterators/SpiralIterator.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/iterators/PolygonIterator.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/iterators/LineIterator.cpp
        ${grid_map_core_SOURCE_DIR}/grid_map_core/src/iterators/SlidingWindowIterator.cpp
    )
    target_include_directories(grid_map_core PUBLIC ${grid_map_core_SOURCE_DIR}/grid_map_core/include)
    target_link_libraries(grid_map_core PUBLIC Eigen3::Eigen)
    target_compile_definitions(grid_map_core PUBLIC
        EIGEN_FUNCTORS_PLUGIN="grid_map_core/eigen_plugins/FunctorsPlugin.hpp"
        EIGEN_DENSEBASE_PLUGIN="grid_map_core/eigen_plugins/DenseBasePlugin.hpp"
    )
    if(MSVC)
        set(GRID_MAP_CORE_MSVC_COMPAT_HEADER
            "${CMAKE_CURRENT_BINARY_DIR}/grid_map_core_msvc_compat.h"
        )
        file(WRITE "${GRID_MAP_CORE_MSVC_COMPAT_HEADER}"
            "#pragma once\n"
            "#ifndef _USE_MATH_DEFINES\n"
            "#define _USE_MATH_DEFINES\n"
            "#endif\n"
            "using uint = unsigned int;\n"
        )
        target_compile_options(grid_map_core PRIVATE
            /FI"${GRID_MAP_CORE_MSVC_COMPAT_HEADER}"
        )
    endif()
endif()

# Fetch GTest for unit testing
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Fetch BehaviorTree.CPP for planner behavior orchestration
FetchContent_Declare(
    behaviortree_cpp
    GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
    GIT_TAG        4.8.2
)
set(BTCPP_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BTCPP_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BTCPP_GROOT_INTERFACE OFF CACHE BOOL "" FORCE)
set(BTCPP_SQLITE_LOGGING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(behaviortree_cpp)

# Fetch nlohmann/json for local API serialization
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

# Fetch cpp-httplib for the local browser visualizer server
FetchContent_Declare(
    cpp_httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG        v0.17.3
)
FetchContent_MakeAvailable(cpp_httplib)

enable_testing()

# Define the static library
add_library(${PROJECT_NAME} STATIC
    src/config/planner_behavior_parser.cpp
    src/config/robots_parser.cpp
    src/math/pose2d.cpp
    src/zones/zone.cpp
    src/zones/maneuvering_zone.cpp
    src/zones/track_main_road.cpp
    src/map/coordinate_transform.cpp
    src/map/map_parser.cpp
    src/robot/car.cpp
    src/robot/truck_trailer.cpp
    src/motion_primitives/car_motion_table.cpp
    src/motion_primitives/truck_trailer_motion_table.cpp
    src/collision_checking/collision_checker.cpp
    src/costs/zone_selector.cpp
    src/costs/static_obstacle_layer.cpp
    src/costs/dynamic_obstacle_layer.cpp
    src/costs/inflation_layer.cpp
    src/costs/map_layer_cache.cpp
    src/costs/zone_constraints_layer.cpp
    src/costs/lane_centerline_layer.cpp
    src/costs/holonomic_obstacles_heuristic.cpp
    src/costs/non_holonomic_heuristic.cpp
    src/costs/dual_model_non_holonomic_heuristic.cpp
    src/costs/costmap_builder.cpp
    src/planning/planner_behavior_profile.cpp
    src/planning/planner_behavior_set.cpp
    src/planning/planner_behavior_resolver.cpp
    src/planning/hybrid_a_star_planner.cpp
    src/planning/behavior_tree_planner_orchestrator.cpp
)

# Specify the include directories for the target
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link Boost and yaml-cpp headers to the target
target_link_libraries(${PROJECT_NAME} PUBLIC 
    Boost::boost 
    yaml-cpp::yaml-cpp
    grid_map_core
    ompl::ompl
)
target_link_libraries(${PROJECT_NAME} PRIVATE behaviortree_cpp)

# Optional: Add compiler warnings for high quality code
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(coast_motion_planning_visualizer STATIC
    src/visualizer/planner_visualizer_service.cpp
    src/visualizer/planner_visualizer_server.cpp
)
target_include_directories(coast_motion_planning_visualizer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(coast_motion_planning_visualizer PUBLIC
    ${PROJECT_NAME}
    httplib::httplib
    nlohmann_json::nlohmann_json
)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(coast_motion_planning_visualizer PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# Subdirectories
# Uncomment these as you add CMakeLists.txt to them.
# ==============================================================================

add_subdirectory(apps)
add_subdirectory(tests)
if(COAST_BUILD_RTMAPS)
    add_subdirectory(rtmaps)
endif()
