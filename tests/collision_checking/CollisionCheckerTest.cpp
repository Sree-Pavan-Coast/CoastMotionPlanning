#include <gtest/gtest.h>
#include <grid_map_core/grid_map_core.hpp>

#include "coastmotionplanning/collision_checking/collision_checker.hpp"
#include "coastmotionplanning/robot/car.hpp"

using namespace coastmotionplanning;

class CollisionCheckerTest : public ::testing::Test {
protected:
    grid_map::GridMap costmap;
    collision_checking::CollisionChecker checker;

    void SetUp() override {
        // 20m x 20m map, 0.1m resolution, centered at origin
        costmap.setGeometry(grid_map::Length(20.0, 20.0), 0.1, grid_map::Position(0.0, 0.0));
        costmap.setFrameId("world");
        costmap.add("obstacle_cost", 0.0); // All free initially
    }
};

// ------------------------------------------------------------------
// Test 1: No obstacles — robot should be collision-free
// ------------------------------------------------------------------
TEST_F(CollisionCheckerTest, NoObstacles_NoCollision) {
    robot::Car car(2.0, 3.0, 0.5, 0.5); // width, wheelbase, front_overhang, rear_overhang
    robot::RobotState state{0.0, 0.0, 0.0, {}};

    auto result = checker.checkCollision(costmap, car, state);

    EXPECT_FALSE(result.in_collision);
    EXPECT_FLOAT_EQ(result.max_cost, 0.0f);
}

// ------------------------------------------------------------------
// Test 2: Place a lethal obstacle directly under the robot
// ------------------------------------------------------------------
TEST_F(CollisionCheckerTest, LethalObstacle_Collision) {
    // Place a lethal obstacle at (1.0, 0.0) — inside the car footprint
    grid_map::Position obstacle_pos(1.0, 0.0);
    grid_map::Index idx;
    ASSERT_TRUE(costmap.getIndex(obstacle_pos, idx));
    costmap.at("obstacle_cost", idx) = 254.0f;

    robot::Car car(2.0, 3.0, 0.5, 0.5);
    robot::RobotState state{0.0, 0.0, 0.0, {}};

    auto result = checker.checkCollision(costmap, car, state);

    EXPECT_TRUE(result.in_collision);
    EXPECT_GE(result.max_cost, 254.0f);
}

// ------------------------------------------------------------------
// Test 3: Obstacle outside the footprint — no collision
// ------------------------------------------------------------------
TEST_F(CollisionCheckerTest, ObstacleOutsideFootprint_NoCollision) {
    // Place obstacle far from the robot
    grid_map::Position obstacle_pos(8.0, 8.0);
    grid_map::Index idx;
    ASSERT_TRUE(costmap.getIndex(obstacle_pos, idx));
    costmap.at("obstacle_cost", idx) = 254.0f;

    robot::Car car(2.0, 3.0, 0.5, 0.5);
    robot::RobotState state{0.0, 0.0, 0.0, {}};

    auto result = checker.checkCollision(costmap, car, state);

    EXPECT_FALSE(result.in_collision);
}

// ------------------------------------------------------------------
// Test 4: Inscribed cost (soft collision below lethal threshold)
// ------------------------------------------------------------------
TEST_F(CollisionCheckerTest, InscribedObstacle_NoBinaryCollision) {
    // Place a mid-cost obstacle under the robot
    grid_map::Position obstacle_pos(1.0, 0.0);
    grid_map::Index idx;
    ASSERT_TRUE(costmap.getIndex(obstacle_pos, idx));
    costmap.at("obstacle_cost", idx) = 128.0f; // Below lethal (254)

    robot::Car car(2.0, 3.0, 0.5, 0.5);
    robot::RobotState state{0.0, 0.0, 0.0, {}};

    auto result = checker.checkCollision(costmap, car, state);

    EXPECT_FALSE(result.in_collision);
    EXPECT_GE(result.max_cost, 128.0f);
}

// ------------------------------------------------------------------
// Test 5: getFootprintCost returns max cost within footprint
// ------------------------------------------------------------------
TEST_F(CollisionCheckerTest, FootprintCost_ReturnsMax) {
    // Scatter some costs under the robot
    for (int i = 0; i < 5; ++i) {
        grid_map::Position pos(static_cast<double>(i) * 0.5, 0.0);
        grid_map::Index idx;
        if (costmap.getIndex(pos, idx)) {
            costmap.at("obstacle_cost", idx) = static_cast<float>(i * 50);
        }
    }

    robot::Car car(2.0, 3.0, 0.5, 0.5);
    robot::RobotState state{0.0, 0.0, 0.0, {}};
    geometry::Polygon2d footprint = car.getRobotFootprint(state);

    float max_cost = checker.getFootprintCost(costmap, footprint);

    EXPECT_GE(max_cost, 200.0f); // i=4 => cost 200
}

// ------------------------------------------------------------------
// Test 6: Point collision check
// ------------------------------------------------------------------
TEST_F(CollisionCheckerTest, PointCollision) {
    grid_map::Position lethal_pos(3.0, 3.0);
    grid_map::Index idx;
    ASSERT_TRUE(costmap.getIndex(lethal_pos, idx));
    costmap.at("obstacle_cost", idx) = 254.0f;

    EXPECT_TRUE(checker.isPointInCollision(costmap, lethal_pos));

    grid_map::Position free_pos(5.0, 5.0);
    EXPECT_FALSE(checker.isPointInCollision(costmap, free_pos));
}

// ------------------------------------------------------------------
// Test 7: Point outside map is treated as collision (conservative)
// ------------------------------------------------------------------
TEST_F(CollisionCheckerTest, PointOutsideMap_IsCollision) {
    grid_map::Position outside_pos(100.0, 100.0);
    EXPECT_TRUE(checker.isPointInCollision(costmap, outside_pos));
}

// ------------------------------------------------------------------
// Test 8: Rotated robot still detects collision correctly
// ------------------------------------------------------------------
TEST_F(CollisionCheckerTest, RotatedRobot_CollisionDetected) {
    // Car: 2m wide, 4m long (wb=3, fo=0.5, ro=0.5). At yaw=PI/2,
    // Boost.Geometry rotate_transformer rotates clockwise, so the car's
    // length axis points along -Y. Place obstacle at (0, -2) which is
    // under the rotated footprint.
    grid_map::Position obstacle_pos(0.0, -2.0);
    grid_map::Index idx;
    ASSERT_TRUE(costmap.getIndex(obstacle_pos, idx));
    costmap.at("obstacle_cost", idx) = 254.0f;

    robot::Car car(2.0, 3.0, 0.5, 0.5);
    robot::RobotState state{0.0, 0.0, M_PI / 2.0, {}}; // 90 degrees

    auto result = checker.checkCollision(costmap, car, state);
    EXPECT_TRUE(result.in_collision);
}

// ------------------------------------------------------------------
// Test 9: Custom collision config with different layer and threshold
// ------------------------------------------------------------------
TEST_F(CollisionCheckerTest, CustomConfig) {
    costmap.add("dynamic_obstacles", 0.0);
    grid_map::Position pos(1.0, 0.0);
    grid_map::Index idx;
    ASSERT_TRUE(costmap.getIndex(pos, idx));
    costmap.at("dynamic_obstacles", idx) = 100.0f;

    collision_checking::CollisionCheckerConfig custom_config;
    custom_config.obstacle_layer = "dynamic_obstacles";
    custom_config.lethal_threshold = 80.0f; // Lower threshold

    collision_checking::CollisionChecker custom_checker(custom_config);
    robot::Car car(2.0, 3.0, 0.5, 0.5);
    robot::RobotState state{0.0, 0.0, 0.0, {}};

    auto result = custom_checker.checkCollision(costmap, car, state);
    EXPECT_TRUE(result.in_collision);
}
