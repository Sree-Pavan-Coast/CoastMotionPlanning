#include <gtest/gtest.h>
#include <chrono>
#include <cmath>

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/costmap_builder.hpp"
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/math/angle.hpp"
#include "coastmotionplanning/robot/car.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

using namespace coastmotionplanning;

class CostmapBuilderTest : public ::testing::Test {
protected:
    std::vector<std::shared_ptr<zones::Zone>> all_zones;
    std::unique_ptr<robot::Car> car;
    costs::CostmapConfig config;

    void SetUp() override {
        // Same zones as largo_map.yaml
        geometry::Polygon2d poly_a;
        poly_a.outer() = {
            geometry::Point2d(-5, -5), geometry::Point2d(15, -5),
            geometry::Point2d(15, 20), geometry::Point2d(-5, 20),
            geometry::Point2d(-5, -5)
        };
        all_zones.push_back(std::make_shared<zones::ManeuveringZone>(poly_a, "zone_a"));

        geometry::Polygon2d poly_b;
        poly_b.outer() = {
            geometry::Point2d(15, 0), geometry::Point2d(100, 0),
            geometry::Point2d(100, 5), geometry::Point2d(15, 5),
            geometry::Point2d(15, 0)
        };
        auto track = std::make_shared<zones::TrackMainRoad>(
            poly_b,
            std::vector<geometry::Point2d>{
                geometry::Point2d(20, 2.5),
                geometry::Point2d(40, 2.5)
            },
            std::vector<double>{1.0, 1.0},
            "zone_b");
        all_zones.push_back(track);

        car = std::make_unique<robot::Car>(2.0, 3.0, 0.5, 0.5);

        config.resolution = 0.1;
        config.inflation_radius_m = 0.5;
        config.inscribed_radius_m = 1.0;
        config.cost_scaling_factor = 3.0;
        config.max_lane_cost = 50.0;
        config.max_lane_half_width = 2.5;
    }
};

TEST_F(CostmapBuilderTest, BuildSameZone) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    math::Pose2d goal(5.0, 5.0, math::Angle::from_radians(0.0));

    auto costmap = builder.build(start, goal);

    // Verify all layers exist
    EXPECT_TRUE(costmap.exists(costs::CostmapLayerNames::STATIC_OBSTACLES));
    EXPECT_TRUE(costmap.exists(costs::CostmapLayerNames::INFLATION));
    EXPECT_TRUE(costmap.exists(costs::CostmapLayerNames::ZONE_CONSTRAINTS));
    EXPECT_TRUE(costmap.exists(costs::CostmapLayerNames::LANE_CENTERLINE_COST));
    EXPECT_TRUE(costmap.exists(costs::CostmapLayerNames::LANE_HEADING));
    EXPECT_TRUE(costmap.exists(costs::CostmapLayerNames::LANE_DISTANCE));
    EXPECT_TRUE(costmap.exists(costs::CostmapLayerNames::TRACK_STATION));
    EXPECT_TRUE(costmap.exists(costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES));
    EXPECT_TRUE(costmap.exists(costs::CostmapLayerNames::COMBINED_COST));
}

TEST_F(CostmapBuilderTest, BuildCrossZone) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));     // zone_a
    math::Pose2d goal(50.0, 2.5, math::Angle::from_radians(0.0));       // zone_b

    auto costmap = builder.build(start, goal);

    // Verify dimensions cover both zones
    const auto& length = costmap.getLength();
    EXPECT_GT(length.x(), 50.0) << "Map should span both zones in X";
}

TEST_F(CostmapBuilderTest, GoalHeuristicIsZero) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    math::Pose2d goal(5.0, 5.0, math::Angle::from_radians(0.0));

    auto costmap = builder.build(start, goal);

    grid_map::Position goal_pos(5.0, 5.0);
    float h = costmap.atPosition(costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES, goal_pos);
    EXPECT_NEAR(h, 0.0f, 0.15f);  // Within one cell resolution
}

TEST_F(CostmapBuilderTest, StartIsFreeCost) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    math::Pose2d goal(5.0, 5.0, math::Angle::from_radians(0.0));

    auto costmap = builder.build(start, goal);

    grid_map::Position start_pos(0.0, 0.0);
    float obs = costmap.atPosition(costs::CostmapLayerNames::STATIC_OBSTACLES, start_pos);
    EXPECT_FLOAT_EQ(obs, costs::CostValues::FREE_SPACE);
}

TEST_F(CostmapBuilderTest, ZoneConstraintsCorrect) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));     // zone_a (maneuvering)
    math::Pose2d goal(50.0, 2.5, math::Angle::from_radians(0.0));       // zone_b (track road)

    auto costmap = builder.build(start, goal);

    // Point in maneuvering zone (zone index 0 — first selected zone)
    grid_map::Position man_pos(0.0, 0.0);
    float zc = costmap.atPosition(costs::CostmapLayerNames::ZONE_CONSTRAINTS, man_pos);
    EXPECT_FLOAT_EQ(zc, 0.0f);  // zone_a is the first selected zone

    // Point in track road zone (zone index 1 — second selected zone)
    grid_map::Position track_pos(50.0, 2.5);
    zc = costmap.atPosition(costs::CostmapLayerNames::ZONE_CONSTRAINTS, track_pos);
    EXPECT_FLOAT_EQ(zc, 1.0f);  // zone_b is the goal frontier
}

TEST_F(CostmapBuilderTest, DisconnectedZonesFailBeforeCostmapBuild) {
    geometry::Polygon2d poly_c;
    poly_c.outer() = {
        geometry::Point2d(120, 0), geometry::Point2d(140, 0),
        geometry::Point2d(140, 10), geometry::Point2d(120, 10),
        geometry::Point2d(120, 0)
    };
    all_zones.push_back(std::make_shared<zones::ManeuveringZone>(poly_c, "zone_c"));

    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    math::Pose2d goal(130.0, 5.0, math::Angle::from_radians(0.0));

    EXPECT_THROW(builder.build(start, goal), std::runtime_error);
}

TEST_F(CostmapBuilderTest, LaneMetadataPrefersForwardLaneTowardTrackGoal) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    math::Pose2d goal(50.0, 2.5, math::Angle::from_radians(0.0));

    auto costmap = builder.build(start, goal);

    const grid_map::Position lower_lane_pos(30.0, 1.5);
    const grid_map::Position upper_lane_pos(30.0, 3.5);
    const grid_map::Position centerline_pos(30.0, 2.5);
    const grid_map::Position maneuver_pos(0.0, 0.0);

    const float lower_heading =
        costmap.atPosition(costs::CostmapLayerNames::LANE_HEADING, lower_lane_pos);
    const float upper_heading =
        costmap.atPosition(costs::CostmapLayerNames::LANE_HEADING, upper_lane_pos);
    const float lower_distance =
        costmap.atPosition(costs::CostmapLayerNames::LANE_DISTANCE, lower_lane_pos);
    const float upper_distance =
        costmap.atPosition(costs::CostmapLayerNames::LANE_DISTANCE, upper_lane_pos);
    const float centerline_distance =
        costmap.atPosition(costs::CostmapLayerNames::LANE_DISTANCE, centerline_pos);
    const float track_station =
        costmap.atPosition(costs::CostmapLayerNames::TRACK_STATION, centerline_pos);
    const float maneuver_heading =
        costmap.atPosition(costs::CostmapLayerNames::LANE_HEADING, maneuver_pos);
    const float maneuver_distance =
        costmap.atPosition(costs::CostmapLayerNames::LANE_DISTANCE, maneuver_pos);
    const float maneuver_station =
        costmap.atPosition(costs::CostmapLayerNames::TRACK_STATION, maneuver_pos);

    EXPECT_NEAR(lower_heading, 0.0f, 1e-3f);
    EXPECT_NEAR(upper_heading, 0.0f, 1e-3f);
    EXPECT_LE(lower_distance, 0.051f);
    EXPECT_NEAR(upper_distance, 2.0f, 0.051f);
    EXPECT_NEAR(centerline_distance, 1.0f, 0.051f);
    EXPECT_NEAR(track_station, 10.0f, 0.051f);
    EXPECT_TRUE(std::isnan(maneuver_heading));
    EXPECT_TRUE(std::isinf(maneuver_distance));
    EXPECT_TRUE(std::isnan(maneuver_station));
}

TEST_F(CostmapBuilderTest, LaneMetadataPrefersReverseLaneWhenGoalStationIsBehind) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(35.0, 2.5, math::Angle::from_degrees(180.0));
    math::Pose2d goal(25.0, 2.5, math::Angle::from_degrees(180.0));

    auto costmap = builder.build(start, goal);

    const grid_map::Position lower_lane_pos(30.0, 1.5);
    const grid_map::Position upper_lane_pos(30.0, 3.5);
    const grid_map::Position centerline_pos(30.0, 2.5);

    const float lower_heading =
        costmap.atPosition(costs::CostmapLayerNames::LANE_HEADING, lower_lane_pos);
    const float upper_heading =
        costmap.atPosition(costs::CostmapLayerNames::LANE_HEADING, upper_lane_pos);
    const float lower_distance =
        costmap.atPosition(costs::CostmapLayerNames::LANE_DISTANCE, lower_lane_pos);
    const float upper_distance =
        costmap.atPosition(costs::CostmapLayerNames::LANE_DISTANCE, upper_lane_pos);
    const float centerline_distance =
        costmap.atPosition(costs::CostmapLayerNames::LANE_DISTANCE, centerline_pos);

    EXPECT_LT(std::cos(lower_heading), -0.99f);
    EXPECT_LT(std::cos(upper_heading), -0.99f);
    EXPECT_NEAR(std::sin(lower_heading), 0.0f, 0.02f);
    EXPECT_NEAR(std::sin(upper_heading), 0.0f, 0.02f);
    EXPECT_NEAR(lower_distance, 2.0f, 0.051f);
    EXPECT_LE(upper_distance, 0.051f);
    EXPECT_NEAR(centerline_distance, 1.0f, 0.051f);
}

TEST_F(CostmapBuilderTest, CrossZoneTrackToManeuverUsesExitDirectionLaneMetadata) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(50.0, 2.5, math::Angle::from_degrees(180.0));
    math::Pose2d goal(0.0, 0.0, math::Angle::from_radians(0.0));

    auto costmap = builder.build(start, goal);

    const grid_map::Position lower_lane_pos(30.0, 1.5);
    const grid_map::Position upper_lane_pos(30.0, 3.5);
    const float lower_distance =
        costmap.atPosition(costs::CostmapLayerNames::LANE_DISTANCE, lower_lane_pos);
    const float upper_distance =
        costmap.atPosition(costs::CostmapLayerNames::LANE_DISTANCE, upper_lane_pos);
    const float upper_heading =
        costmap.atPosition(costs::CostmapLayerNames::LANE_HEADING, upper_lane_pos);

    EXPECT_NEAR(lower_distance, 2.0f, 0.051f);
    EXPECT_LE(upper_distance, 0.051f);
    EXPECT_LT(std::cos(upper_heading), -0.99f);
    EXPECT_NEAR(std::sin(upper_heading), 0.0f, 0.02f);
}

TEST_F(CostmapBuilderTest, RebuildWithUpdatedObstaclePolygonsResetsDerivedLayers) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    const math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    const math::Pose2d goal(10.0, 0.0, math::Angle::from_radians(0.0));
    const grid_map::Position wall_center(5.0, 0.0);
    const grid_map::Position inflated_sample(2.7, 0.0);
    const grid_map::Position start_pos(start.x, start.y);

    geometry::Polygon2d blocking_wall;
    blocking_wall.outer() = {
        geometry::Point2d(3.0, -10.0), geometry::Point2d(7.0, -10.0),
        geometry::Point2d(7.0, 25.0), geometry::Point2d(3.0, 25.0),
        geometry::Point2d(3.0, -10.0)
    };

    const auto baseline_costmap = builder.build(start, goal);
    const float baseline_static =
        baseline_costmap.atPosition(costs::CostmapLayerNames::STATIC_OBSTACLES, wall_center);
    const float baseline_heuristic =
        baseline_costmap.atPosition(costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES, start_pos);

    const auto blocked_costmap = builder.build(start, goal, {blocking_wall});
    const float blocked_static =
        blocked_costmap.atPosition(costs::CostmapLayerNames::STATIC_OBSTACLES, wall_center);
    const float blocked_inflation =
        blocked_costmap.atPosition(costs::CostmapLayerNames::INFLATION, inflated_sample);
    const float blocked_heuristic =
        blocked_costmap.atPosition(costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES, start_pos);

    const auto restored_costmap = builder.build(start, goal);
    const float restored_static =
        restored_costmap.atPosition(costs::CostmapLayerNames::STATIC_OBSTACLES, wall_center);
    const float restored_heuristic =
        restored_costmap.atPosition(costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES, start_pos);

    EXPECT_FLOAT_EQ(baseline_static, costs::CostValues::FREE_SPACE);
    EXPECT_TRUE(std::isfinite(baseline_heuristic));

    EXPECT_FLOAT_EQ(blocked_static, costs::CostValues::LETHAL);
    EXPECT_GT(blocked_inflation, costs::CostValues::FREE_SPACE);
    EXPECT_TRUE(std::isnan(blocked_heuristic));

    EXPECT_FLOAT_EQ(restored_static, costs::CostValues::FREE_SPACE);
    EXPECT_TRUE(std::isfinite(restored_heuristic));
}

TEST_F(CostmapBuilderTest, PerformanceBudget) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    math::Pose2d goal(50.0, 2.5, math::Angle::from_radians(0.0));

    auto t_start = std::chrono::high_resolution_clock::now();
    auto costmap = builder.build(start, goal);
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - t_start);

    std::cout << "[PROFILE] Full costmap build: " << elapsed.count() << " ms" << std::endl;
    std::cout << "[PROFILE] Grid size: " << costmap.getSize()(0) << " x " 
              << costmap.getSize()(1) << " = " 
              << costmap.getSize()(0) * costmap.getSize()(1) << " cells" << std::endl;

    // Budget: should complete within a reasonable time for this grid size
    EXPECT_LT(elapsed.count(), 30000) << "Full build should complete within 30s";
}
