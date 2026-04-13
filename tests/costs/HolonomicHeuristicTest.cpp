#include <gtest/gtest.h>
#include <chrono>

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/static_obstacle_layer.hpp"
#include "coastmotionplanning/costs/holonomic_obstacles_heuristic.hpp"
#include "coastmotionplanning/costs/costmap_types.hpp"

using namespace coastmotionplanning;

class HolonomicHeuristicTest : public ::testing::Test {
protected:
    grid_map::GridMap costmap;

    void SetUp() override {
        costmap.setGeometry(grid_map::Length(20.0, 20.0), 0.1, grid_map::Position(0, 0));
        costmap.setFrameId("world");

        geometry::Polygon2d boundary;
        boundary.outer() = {
            geometry::Point2d(-9.5, -9.5), geometry::Point2d(9.5, -9.5),
            geometry::Point2d(9.5, 9.5), geometry::Point2d(-9.5, 9.5),
            geometry::Point2d(-9.5, -9.5)
        };
        costs::StaticObstacleLayer::build(costmap, boundary);
    }
};

TEST_F(HolonomicHeuristicTest, GoalCellIsZero) {
    grid_map::Position goal(0.0, 0.0);
    costs::HolonomicObstaclesHeuristic::compute(costmap, goal);

    float dist = costmap.atPosition(costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES, goal);
    EXPECT_FLOAT_EQ(dist, 0.0f);
}

TEST_F(HolonomicHeuristicTest, FreeSpaceApproximatesEuclidean) {
    grid_map::Position goal(0.0, 0.0);
    costs::HolonomicObstaclesHeuristic::compute(costmap, goal);

    // Point 5m away in clear space
    grid_map::Position test_pos(5.0, 0.0);
    float heuristic = costmap.atPosition(costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES, test_pos);

    // Should be close to Euclidean distance (5.0m), within grid resolution tolerance
    EXPECT_NEAR(heuristic, 5.0, 0.2);
}

TEST_F(HolonomicHeuristicTest, ObstacleCreatesLongerPath) {
    // Place a wall of obstacles across the middle
    for (double y = -5.0; y <= 5.0; y += 0.1) {
        grid_map::Position wall_pos(3.0, y);
        grid_map::Index idx;
        if (costmap.getIndex(wall_pos, idx)) {
            costmap.at(costs::CostmapLayerNames::STATIC_OBSTACLES, idx) = costs::CostValues::LETHAL;
        }
    }

    grid_map::Position goal(0.0, 0.0);  // Left of wall
    costs::HolonomicObstaclesHeuristic::compute(costmap, goal);

    // Point on the other side of the wall
    grid_map::Position test_pos(5.0, 0.0);
    float heuristic = costmap.atPosition(costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES, test_pos);

    // Must go around the wall, so distance > Euclidean (5.0m)
    float euclidean = 5.0f;
    EXPECT_GT(heuristic, euclidean) << "Heuristic should be > Euclidean due to wall";
}

TEST_F(HolonomicHeuristicTest, UnreachableCellIsNaN) {
    // Create a watertight enclosure by directly iterating grid indices.
    // Build a thick rectangular frame from row/col 50-55 and 145-150 (in a 200x200 grid)
    auto& obstacles = costmap[costs::CostmapLayerNames::STATIC_OBSTACLES];
    int rows = costmap.getSize()(0);
    int cols = costmap.getSize()(1);

    // Horizontal walls: rows [50,55] and [145,150], full width
    for (int r = 50; r <= 55; ++r) {
        for (int c = 50; c <= 150; ++c) {
            obstacles(r, c) = costs::CostValues::LETHAL;
        }
    }
    for (int r = 145; r <= 150; ++r) {
        for (int c = 50; c <= 150; ++c) {
            obstacles(r, c) = costs::CostValues::LETHAL;
        }
    }
    // Vertical walls: cols [50,55] and [145,150], between the horizontals
    for (int r = 50; r <= 150; ++r) {
        for (int c = 50; c <= 55; ++c) {
            obstacles(r, c) = costs::CostValues::LETHAL;
        }
        for (int c = 145; c <= 150; ++c) {
            obstacles(r, c) = costs::CostValues::LETHAL;
        }
    }

    // Goal inside the box (center of [56,144] x [56,144] region = row 100, col 100)
    grid_map::Position goal(0.0, 0.0);  // Center of grid
    costs::HolonomicObstaclesHeuristic::compute(costmap, goal);

    // Check a cell outside the box — should be NaN
    // Row 10, Col 10 is well outside (top-left area outside the boundary ring)
    float val = obstacles(10, 10);
    // First verify this outside cell is free space (inside boundary but outside box)
    ASSERT_LT(val, costs::CostValues::LETHAL) << "Test point should not be lethal itself";

    float h = costmap[costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES](10, 10);
    EXPECT_TRUE(std::isnan(h)) << "Cell outside sealed box should be NaN, got: " << h;
}

TEST_F(HolonomicHeuristicTest, PerformanceBudget) {
    grid_map::GridMap large_map;
    large_map.setGeometry(grid_map::Length(100.0, 100.0), 0.1, grid_map::Position(0, 0));
    large_map.setFrameId("world");

    geometry::Polygon2d large_boundary;
    large_boundary.outer() = {
        geometry::Point2d(-49.0, -49.0), geometry::Point2d(49.0, -49.0),
        geometry::Point2d(49.0, 49.0), geometry::Point2d(-49.0, 49.0),
        geometry::Point2d(-49.0, -49.0)
    };
    costs::StaticObstacleLayer::build(large_map, large_boundary);

    auto start = std::chrono::high_resolution_clock::now();
    costs::HolonomicObstaclesHeuristic::compute(large_map, grid_map::Position(0, 0));
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);

    std::cout << "[PROFILE] Dijkstra on 1M cells: " << elapsed.count() << " ms" << std::endl;
    EXPECT_LT(elapsed.count(), 5000) << "Dijkstra should complete within 5000ms for 1M cells";
}
