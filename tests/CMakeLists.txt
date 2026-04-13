# ==============================================================================
# Tests Configuration
# ==============================================================================

# Include GoogleTest module
include(GoogleTest)

# Define the unit test executable
add_executable(grid_map_test planning/GridMapTest.cpp)
target_link_libraries(grid_map_test PRIVATE 
    gtest_main
    coast_motion_planning
    grid_map_core
)
gtest_discover_tests(grid_map_test)

# Map Parser and Coordinate Transform Tests
add_executable(map_parser_test map/MapParserTest.cpp)
target_link_libraries(map_parser_test PRIVATE 
    gtest_main
    coast_motion_planning
    grid_map_core
    yaml-cpp::yaml-cpp
)
gtest_discover_tests(map_parser_test)

# Config parser tests
add_executable(robots_parser_test config/RobotsParserTest.cpp)
target_link_libraries(robots_parser_test PRIVATE
    gtest_main
    coast_motion_planning
    yaml-cpp::yaml-cpp
)
gtest_discover_tests(robots_parser_test)

add_executable(planner_behavior_parser_test config/PlannerBehaviorParserTest.cpp)
target_link_libraries(planner_behavior_parser_test PRIVATE
    gtest_main
    coast_motion_planning
    yaml-cpp::yaml-cpp
)
gtest_discover_tests(planner_behavior_parser_test)

# Motion Primitives Tests
add_executable(motion_primitives_test motion/MotionPrimitivesTest.cpp)
target_link_libraries(motion_primitives_test PRIVATE
    gtest_main
    coast_motion_planning
)
gtest_discover_tests(motion_primitives_test)

# Collision Checker Tests
add_executable(collision_checker_test collision_checking/CollisionCheckerTest.cpp)
target_link_libraries(collision_checker_test PRIVATE 
    gtest_main
    coast_motion_planning
    grid_map_core
)
gtest_discover_tests(collision_checker_test)

# Costmap Layer Tests
set(COSTS_LINK_LIBS gtest_main coast_motion_planning grid_map_core)

add_executable(zone_selector_test costs/ZoneSelectorTest.cpp)
target_link_libraries(zone_selector_test PRIVATE ${COSTS_LINK_LIBS})
gtest_discover_tests(zone_selector_test)

add_executable(inflation_layer_test costs/InflationLayerTest.cpp)
target_link_libraries(inflation_layer_test PRIVATE ${COSTS_LINK_LIBS})
gtest_discover_tests(inflation_layer_test)

add_executable(holonomic_heuristic_test costs/HolonomicHeuristicTest.cpp)
target_link_libraries(holonomic_heuristic_test PRIVATE ${COSTS_LINK_LIBS})
gtest_discover_tests(holonomic_heuristic_test)

add_executable(non_holonomic_heuristic_test costs/NonHolonomicHeuristicTest.cpp)
target_link_libraries(non_holonomic_heuristic_test PRIVATE ${COSTS_LINK_LIBS})
gtest_discover_tests(non_holonomic_heuristic_test)

add_executable(dual_model_non_holonomic_heuristic_test costs/DualModelNonHolonomicHeuristicTest.cpp)
target_link_libraries(dual_model_non_holonomic_heuristic_test PRIVATE ${COSTS_LINK_LIBS})
gtest_discover_tests(dual_model_non_holonomic_heuristic_test)

add_executable(costmap_builder_test costs/CostmapBuilderTest.cpp)
target_link_libraries(costmap_builder_test PRIVATE ${COSTS_LINK_LIBS})
gtest_discover_tests(costmap_builder_test)

add_executable(behavior_tree_planner_test planning/BehaviorTreePlannerTest.cpp)
target_link_libraries(behavior_tree_planner_test PRIVATE gtest_main coast_motion_planning)
target_compile_definitions(
    behavior_tree_planner_test
    PRIVATE COAST_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
gtest_discover_tests(behavior_tree_planner_test)

add_executable(planner_behavior_resolver_test planning/PlannerBehaviorResolverTest.cpp)
target_link_libraries(planner_behavior_resolver_test PRIVATE gtest_main coast_motion_planning)
target_compile_definitions(
    planner_behavior_resolver_test
    PRIVATE COAST_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
gtest_discover_tests(planner_behavior_resolver_test)

add_executable(hybrid_a_star_planner_test planning/HybridAStarPlannerTest.cpp)
target_link_libraries(hybrid_a_star_planner_test PRIVATE gtest_main coast_motion_planning)
target_compile_definitions(
    hybrid_a_star_planner_test
    PRIVATE COAST_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
gtest_discover_tests(hybrid_a_star_planner_test)
