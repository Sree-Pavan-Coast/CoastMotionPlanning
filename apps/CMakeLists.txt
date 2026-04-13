# ==============================================================================
# Applications
# ==============================================================================

add_executable(generate_nh_lut generate_nh_lut.cpp)
target_link_libraries(generate_nh_lut PRIVATE coast_motion_planning grid_map_core)
