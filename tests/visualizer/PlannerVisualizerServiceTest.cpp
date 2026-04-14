#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <string>

#include "coastmotionplanning/visualizer/planner_visualizer_service.hpp"

namespace {

using coastmotionplanning::visualizer::MapLoadRequest;
using coastmotionplanning::visualizer::PlanRequest;
using coastmotionplanning::visualizer::PlannerVisualizerService;
using coastmotionplanning::visualizer::PlannerVisualizerServiceConfig;
using coastmotionplanning::visualizer::PoseDto;

std::string configsRoot() {
    return std::string(COAST_REPO_ROOT) + "/configs";
}

std::string simpleMapYaml() {
    return R"(
maps:
  name: "Visualizer Test Map"
zones:
  - name: "track_zone"
    type: "TrackMainRoad"
    planner_behavior: "primary_profile"
    coordinate_type: "world"
    polygon:
      - [0.0, -4.0]
      - [12.0, -4.0]
      - [12.0, 4.0]
      - [0.0, 4.0]
    lanes:
      - lane_waypoints:
          - [1.0, 0.0]
          - [11.0, 0.0]
)";
}

std::string maneuveringMapYaml() {
    return R"(
maps:
  name: "Visualizer Maneuvering Map"
zones:
  - name: "maneuver_zone"
    type: "ManeuveringZone"
    planner_behavior: "parking_profile"
    coordinate_type: "world"
    polygon:
      - [0.0, 0.0]
      - [12.0, 0.0]
      - [12.0, 10.0]
      - [0.0, 10.0]
)";
}

PlannerVisualizerService makeService() {
    return PlannerVisualizerService(PlannerVisualizerServiceConfig{
        configsRoot(),
        "Pro_XD"
    });
}

TEST(PlannerVisualizerServiceTest, MakePoseNormalizesHeadingDegrees) {
    const auto pose = PlannerVisualizerService::makePose(PoseDto{1.5, -2.0, 450.0}, "start");
    EXPECT_DOUBLE_EQ(pose.x, 1.5);
    EXPECT_DOUBLE_EQ(pose.y, -2.0);
    EXPECT_NEAR(pose.theta.degrees(), 90.0, 1e-6);
}

TEST(PlannerVisualizerServiceTest, LoadMapBuildsGeometryAndBounds) {
    auto service = makeService();

    const auto response = service.loadMap(MapLoadRequest{
        "test_map.yaml",
        simpleMapYaml(),
        ""
    });

    EXPECT_EQ(response.map_name, "Visualizer Test Map");
    EXPECT_FALSE(response.map_id.empty());
    EXPECT_EQ(response.bounds.min_x, 0.0);
    EXPECT_EQ(response.bounds.max_x, 12.0);
    EXPECT_EQ(response.bounds.min_y, -4.0);
    EXPECT_EQ(response.bounds.max_y, 4.0);
    EXPECT_EQ(response.vehicle.name, "Pro_XD");
    EXPECT_GT(response.vehicle.width_m, 0.0);
    EXPECT_GT(response.vehicle.total_length_m, response.vehicle.wheelbase_m);
    EXPECT_GT(response.vehicle.min_turning_radius_m, 0.0);
    EXPECT_GT(response.vehicle.max_steer_angle_rad, 0.0);
    ASSERT_EQ(response.zones.size(), 1u);
    EXPECT_EQ(response.zones[0].type, "TrackMainRoad");
    ASSERT_EQ(response.zones[0].lanes.size(), 1u);
    ASSERT_EQ(response.zones[0].lanes[0].poses.size(), 2u);
}

TEST(PlannerVisualizerServiceTest, ListsRobotsFromRobotCatalogAndFlagsUnsupportedEntries) {
    auto service = makeService();

    const auto catalog = service.listRobots();

    EXPECT_EQ(catalog.default_robot_name, "Pro_XD");
    ASSERT_GE(catalog.robots.size(), 4u);
    EXPECT_EQ(catalog.robots.front().name, "Pro_XD");
    EXPECT_EQ(catalog.robots.front().type, "Car");
    EXPECT_TRUE(catalog.robots.front().planning_supported);
    EXPECT_GT(catalog.robots.front().vehicle.min_turning_radius_m, 0.0);

    const auto unsupported = std::find_if(
        catalog.robots.begin(),
        catalog.robots.end(),
        [](const auto& robot) { return robot.type == "TruckTrailer"; });
    ASSERT_NE(unsupported, catalog.robots.end());
    EXPECT_FALSE(unsupported->planning_supported);
}

TEST(PlannerVisualizerServiceTest, LoadMapUsesSelectedRobotKinematics) {
    auto service = makeService();

    const auto response = service.loadMap(MapLoadRequest{
        "test_map.yaml",
        simpleMapYaml(),
        "E_Transit"
    });

    EXPECT_EQ(response.vehicle.name, "E_Transit");
    EXPECT_NEAR(response.vehicle.min_turning_radius_m, 7.0715315097, 1e-6);
    EXPECT_NEAR(response.vehicle.max_steer_angle_rad, 0.4886921906, 1e-6);
}

TEST(PlannerVisualizerServiceTest, PlanReturnsPathForValidScenario) {
    auto service = makeService();
    const auto map = service.loadMap(MapLoadRequest{"test_map.yaml", simpleMapYaml(), "Pro_XD"});

    const auto result = service.plan(PlanRequest{
        map.map_id,
        "Pro_XD",
        PoseDto{2.0, 0.0, 0.0},
        PoseDto{7.0, 0.0, 0.0}
    });

    EXPECT_TRUE(result.success) << result.detail;
    EXPECT_EQ(result.selected_profile, "primary_profile");
    EXPECT_FALSE(result.attempted_profiles.empty());
    EXPECT_FALSE(result.path.empty());
    EXPECT_EQ(result.segment_directions.size(), result.path.size() - 1);
}

TEST(PlannerVisualizerServiceTest, OppositeHeadingFailureIncludesTurningRadiusGuidance) {
    auto service = makeService();
    const auto map = service.loadMap(
        MapLoadRequest{"maneuvering_map.yaml", maneuveringMapYaml(), "Pro_XD"});

    const auto result = service.plan(PlanRequest{
        map.map_id,
        "Pro_XD",
        PoseDto{2.0, 5.0, 0.0},
        PoseDto{6.0, 5.0, 180.0}
    });

    EXPECT_FALSE(result.success);
    EXPECT_NE(result.detail.find("turning radius"), std::string::npos);
    EXPECT_NE(result.detail.find("maneuvering zone"), std::string::npos);
}

TEST(PlannerVisualizerServiceTest, PlanThrowsOnUnknownMapId) {
    auto service = makeService();

    EXPECT_THROW(
        service.plan(PlanRequest{
            "missing-map",
            "Pro_XD",
            PoseDto{0.0, 0.0, 0.0},
            PoseDto{1.0, 1.0, 0.0}
        }),
        std::runtime_error);
}

} // namespace
