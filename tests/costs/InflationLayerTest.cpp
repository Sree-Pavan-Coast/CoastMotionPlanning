#include <gtest/gtest.h>
#include <chrono>

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/static_obstacle_layer.hpp"
#include "coastmotionplanning/costs/inflation_layer.hpp"
#include "coastmotionplanning/costs/costmap_types.hpp"

using namespace coastmotionplanning;

class InflationLayerTest : public ::testing::Test {
protected:
    grid_map::GridMap costmap;
    geometry::Polygon2d boundary;

    void SetUp() override {
        // 20m x 20m map at 0.1m resolution
        costmap.setGeometry(grid_map::Length(20.0, 20.0), 0.1, grid_map::Position(0, 0));
        costmap.setFrameId("world");

        // Create a simple square boundary
        boundary.outer() = {
            geometry::Point2d(-9.0, -9.0), geometry::Point2d(9.0, -9.0),
            geometry::Point2d(9.0, 9.0), geometry::Point2d(-9.0, 9.0),
            geometry::Point2d(-9.0, -9.0)
        };

        // Build static obstacles first
        costs::StaticObstacleLayer::build(costmap, boundary);
    }
};

TEST_F(InflationLayerTest, MonotonicGradient) {
    costs::InflationLayer::build(costmap, 2.0, 0.5, 3.0);

    // Check gradient decreases monotonically along a line from a lethal cell
    // The boundary edge at x=9.0 creates lethal cells just outside
    // Walk inward from x=8.9 (near boundary) toward center
    float prev_cost = std::numeric_limits<float>::max();
    for (double x = 8.9; x > 5.0; x -= 0.1) {
        grid_map::Position pos(x, 0.0);
        if (costmap.isInside(pos)) {
            float cost = costmap.atPosition(costs::CostmapLayerNames::INFLATION, pos);
            EXPECT_LE(cost, prev_cost) << "Cost should decrease at x=" << x;
            prev_cost = cost;
        }
    }
}

TEST_F(InflationLayerTest, BeyondInflationRadiusIsZero) {
    costs::InflationLayer::build(costmap, 1.0, 0.3, 3.0);

    // Center of map should be far from any obstacle, so inflation cost = 0
    grid_map::Position center(0.0, 0.0);
    float cost = costmap.atPosition(costs::CostmapLayerNames::INFLATION, center);
    EXPECT_FLOAT_EQ(cost, costs::CostValues::FREE_SPACE);
}

TEST_F(InflationLayerTest, WithinInscribedRadiusIsInscribed) {
    costs::InflationLayer::build(costmap, 2.0, 0.5, 3.0);

    // Very close to the boundary should be INSCRIBED
    // The boundary is at x=9.0, lethal cells are beyond that.
    // At x=8.8 (0.2m from boundary), should be within inscribed radius of 0.5m
    grid_map::Position near_wall(8.8, 0.0);
    if (costmap.isInside(near_wall)) {
        float cost = costmap.atPosition(costs::CostmapLayerNames::INFLATION, near_wall);
        // Should be at or near INSCRIBED since we're well within inscribed radius
        EXPECT_GE(cost, costs::CostValues::INSCRIBED * 0.5f);
    }
}

TEST_F(InflationLayerTest, MultipleObstaclesMerge) {
    // Add an obstacle in the center
    grid_map::Position obstacle_pos(0.0, 0.0);
    grid_map::Index idx;
    ASSERT_TRUE(costmap.getIndex(obstacle_pos, idx));
    costmap.at(costs::CostmapLayerNames::STATIC_OBSTACLES, idx) = costs::CostValues::LETHAL;

    costs::InflationLayer::build(costmap, 1.5, 0.3, 3.0);

    // Check that inflation extends around the center obstacle
    grid_map::Position near_center(0.5, 0.0);
    float cost = costmap.atPosition(costs::CostmapLayerNames::INFLATION, near_center);
    EXPECT_GT(cost, 0.0f) << "Should have inflation cost near center obstacle";
}

TEST_F(InflationLayerTest, PerformanceBudget) {
    // Test with a larger map: 100x100m at 0.1m = 1M cells
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
    costs::InflationLayer::build(large_map, 2.0, 0.5, 3.0);
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);

    std::cout << "[PROFILE] Inflation on 1M cells: " << elapsed.count() << " ms" << std::endl;
    EXPECT_LT(elapsed.count(), 2000) << "Inflation should complete within 2000ms for 1M cells";
}
