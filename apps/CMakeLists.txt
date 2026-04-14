# ==============================================================================
# Applications
# ==============================================================================

add_executable(generate_nh_lut generate_nh_lut.cpp)
target_link_libraries(generate_nh_lut PRIVATE coast_motion_planning grid_map_core)

add_executable(planner_visualizer planner_visualizer.cpp)
target_link_libraries(planner_visualizer PRIVATE coast_motion_planning_visualizer)
target_compile_definitions(
    planner_visualizer
    PRIVATE
        COAST_REPO_ROOT="${CMAKE_SOURCE_DIR}"
        COAST_PLANNER_VISUALIZER_ASSET_DIR="${CMAKE_SOURCE_DIR}/apps/planner_visualizer"
)