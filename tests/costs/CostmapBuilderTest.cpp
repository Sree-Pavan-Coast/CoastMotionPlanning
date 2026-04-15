#include <gtest/gtest.h>
#include <chrono>
#include <cmath>

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/costmap_builder.hpp"
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/costs/holonomic_obstacles_heuristic.hpp"
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
        auto track = std::make_shared<zones::TrackMainRoad>(poly_b, "zone_b");
        std::vector<geometry::Point2d> lane_a = {
            geometry::Point2d(20, 1.5), geometry::Point2d(40, 1.5)
        };
        std::vector<geometry::Point2d> lane_b = {
            geometry::Point2d(40, 3.5), geometry::Point2d(20, 3.5)
        };
        track->addLaneFromPoints(lane_a);
        track->addLaneFromPoints(lane_b);
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
    EXPECT_TRUE(costmap.exists(costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES));
    EXPECT_TRUE(costmap.exists(costs::CostmapLayerNames::COMBINED_COST));
    EXPECT_TRUE(builder.getStageHeuristicLayerSummaries().empty());
}

TEST_F(CostmapBuilderTest, BuildCrossZone) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));     // zone_a
    math::Pose2d goal(50.0, 2.5, math::Angle::from_radians(0.0));       // zone_b

    auto costmap = builder.build(start, goal);

    // Verify dimensions cover both zones
    const auto& length = costmap.getLength();
    EXPECT_GT(length.x(), 50.0) << "Map should span both zones in X";
    const auto& stage_summaries = builder.getStageHeuristicLayerSummaries();
    ASSERT_EQ(stage_summaries.size(), 2u);
    EXPECT_TRUE(costmap.exists(
        costs::HolonomicObstaclesHeuristic::makeStageLayerName(0, 1)));
    EXPECT_TRUE(costmap.exists(
        costs::HolonomicObstaclesHeuristic::makeStageLayerName(1, 2)));
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
    EXPECT_FLOAT_EQ(zc, 2.0f);  // zone_b is the goal frontier when a transition frontier exists
}

TEST_F(CostmapBuilderTest, GapBetweenZonesBecomesDrivableTransitionSpace) {
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

    auto costmap = builder.build(start, goal);

    const grid_map::Position gap_pos(70.0, 5.0);
    const float zone_value =
        costmap.atPosition(costs::CostmapLayerNames::ZONE_CONSTRAINTS, gap_pos);
    const float combined_cost =
        costmap.atPosition(costs::CostmapLayerNames::COMBINED_COST, gap_pos);
    const float static_cost =
        costmap.atPosition(costs::CostmapLayerNames::STATIC_OBSTACLES, gap_pos);

    EXPECT_FLOAT_EQ(zone_value, 1.0f);
    EXPECT_FLOAT_EQ(static_cost, costs::CostValues::FREE_SPACE);
    EXPECT_LT(combined_cost, costs::CostValues::LETHAL);
}

TEST_F(CostmapBuilderTest, StageHeuristicLayersSeedFromNextFrontierRegions) {
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

    auto costmap = builder.build(start, goal);

    const std::string start_stage_layer =
        costs::HolonomicObstaclesHeuristic::makeStageLayerName(0, 1);
    const std::string transition_stage_layer =
        costs::HolonomicObstaclesHeuristic::makeStageLayerName(1, 2);

    const grid_map::Position gap_pos(70.0, 5.0);
    const grid_map::Position goal_zone_pos(130.0, 5.0);
    EXPECT_NEAR(costmap.atPosition(start_stage_layer, gap_pos), 0.0f, 0.15f);
    EXPECT_NEAR(costmap.atPosition(transition_stage_layer, goal_zone_pos), 0.0f, 0.15f);

    const auto& stage_summaries = builder.getStageHeuristicLayerSummaries();
    ASSERT_EQ(stage_summaries.size(), 2u);
    EXPECT_GT(stage_summaries[0].seed_cell_count, 0u);
    EXPECT_GT(stage_summaries[1].seed_cell_count, 0u);
}

TEST_F(CostmapBuilderTest, LaneMetadataLayersEncodeDirectionalHeadingAndDistance) {
    costs::CostmapBuilder builder(config, all_zones, *car);

    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    math::Pose2d goal(50.0, 2.5, math::Angle::from_radians(0.0));

    auto costmap = builder.build(start, goal);

    const grid_map::Position lower_lane_pos(30.0, 1.5);
    const grid_map::Position upper_lane_pos(30.0, 3.5);
    const grid_map::Position maneuver_pos(0.0, 0.0);

    const float lower_heading =
        costmap.atPosition(costs::CostmapLayerNames::LANE_HEADING, lower_lane_pos);
    const float upper_heading =
        costmap.atPosition(costs::CostmapLayerNames::LANE_HEADING, upper_lane_pos);
    const float lower_distance =
        costmap.atPosition(costs::CostmapLayerNames::LANE_DISTANCE, lower_lane_pos);
    const float maneuver_heading =
        costmap.atPosition(costs::CostmapLayerNames::LANE_HEADING, maneuver_pos);
    const float maneuver_distance =
        costmap.atPosition(costs::CostmapLayerNames::LANE_DISTANCE, maneuver_pos);

    EXPECT_NEAR(lower_heading, 0.0f, 1e-3f);
    EXPECT_NEAR(std::abs(upper_heading), 3.14159265f, 1e-3f);
    EXPECT_LE(lower_distance, 0.051f);
    EXPECT_TRUE(std::isnan(maneuver_heading));
    EXPECT_TRUE(std::isinf(maneuver_distance));
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
