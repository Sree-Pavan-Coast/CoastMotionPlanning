#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "coastmotionplanning/visualizer/planner_visualizer_service.hpp"

namespace {

using json = nlohmann::json;
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
          - [1.0, -1.5]
          - [11.0, -1.5]
      - lane_waypoints:
          - [11.0, 1.5]
          - [1.0, 1.5]
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
      - [6.0, 0.0]
      - [6.0, 6.0]
      - [0.0, 6.0]
)";
}

std::string retryManeuveringMapYaml() {
    return R"(
maps:
  name: "Visualizer Retry Maneuvering Map"
zones:
  - name: "maneuver_zone"
    type: "ManeuveringZone"
    planner_behavior: "primary_profile"
    coordinate_type: "world"
    polygon:
      - [-5.0, -5.0]
      - [15.0, -5.0]
      - [15.0, 20.0]
      - [-5.0, 20.0]
)";
}

std::string readTextFile(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream.is_open()) {
        throw std::runtime_error("Failed to read file: " + path.string());
    }
    return std::string(
        std::istreambuf_iterator<char>(stream),
        std::istreambuf_iterator<char>());
}

std::filesystem::path makeTempConfigsRoot(bool debug_mode,
                                          bool primary_only_forward = false) {
    const auto unique_id = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto temp_root = std::filesystem::temp_directory_path() /
        ("coastmotionplanning_visualizer_service_test_" + std::to_string(unique_id));
    const auto temp_configs_root = temp_root / "configs";
    std::filesystem::create_directories(temp_root);
    std::filesystem::copy(
        configsRoot(),
        temp_configs_root,
        std::filesystem::copy_options::recursive);

    std::string behaviors = readTextFile(temp_configs_root / "planner_behaviors.yaml");
    if (debug_mode) {
        const std::string debug_mode_disabled = "  debug_mode: false";
        const size_t debug_mode_pos = behaviors.find(debug_mode_disabled);
        if (debug_mode_pos == std::string::npos) {
            throw std::runtime_error(
                "Failed to locate global.debug_mode in planner_behaviors.yaml");
        }
        behaviors.replace(
            debug_mode_pos,
            debug_mode_disabled.size(),
            "  debug_mode: true");
    }

    if (primary_only_forward) {
        const size_t primary_profile_pos = behaviors.find("  primary_profile:\n");
        if (primary_profile_pos == std::string::npos) {
            throw std::runtime_error(
                "Failed to locate primary_profile in planner_behaviors.yaml");
        }
        const std::string only_forward_false = "      only_forward_path: false";
        const size_t only_forward_pos =
            behaviors.find(only_forward_false, primary_profile_pos);
        if (only_forward_pos == std::string::npos) {
            throw std::runtime_error(
                "Failed to locate primary_profile.only_forward_path");
        }
        behaviors.replace(
            only_forward_pos,
            only_forward_false.size(),
            "      only_forward_path: true");
    }

    std::ofstream behavior_stream(temp_configs_root / "planner_behaviors.yaml");
    behavior_stream << behaviors;
    behavior_stream.close();
    return temp_configs_root;
}

json loadJsonFile(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream.is_open()) {
        throw std::runtime_error("Failed to open JSON file: " + path.string());
    }
    return json::parse(stream);
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
    ASSERT_EQ(response.zones[0].lanes.size(), 2u);
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

TEST(PlannerVisualizerServiceTest, DebugModeWritesProfilingSummaryIntoReport) {
    auto service = PlannerVisualizerService(PlannerVisualizerServiceConfig{
        makeTempConfigsRoot(true),
        "Pro_XD"
    });
    const auto map = service.loadMap(MapLoadRequest{"test_map.yaml", simpleMapYaml(), "Pro_XD"});

    const auto result = service.plan(PlanRequest{
        map.map_id,
        "Pro_XD",
        PoseDto{2.0, 0.0, 0.0},
        PoseDto{7.0, 0.0, 0.0}
    });

    ASSERT_TRUE(result.success) << result.detail;
    EXPECT_TRUE(result.debug_mode);
    ASSERT_FALSE(result.debug_report_path.empty());
    ASSERT_TRUE(std::filesystem::exists(result.debug_report_path));

    const json report = loadJsonFile(result.debug_report_path);
    ASSERT_TRUE(report.contains("attempts"));
    ASSERT_EQ(report.at("attempts").size(), 1u);

    const json& attempt = report.at("attempts").at(0);
    ASSERT_TRUE(attempt.contains("profiling"));
    ASSERT_TRUE(attempt.at("profiling").contains("scopes"));
    ASSERT_FALSE(attempt.at("profiling").at("scopes").empty());
    ASSERT_TRUE(attempt.contains("debug_trace"));
    ASSERT_TRUE(attempt.at("debug_trace").contains("profiling"));
    ASSERT_TRUE(attempt.at("debug_trace").at("profiling").contains("scopes"));

    const auto& scopes = attempt.at("profiling").at("scopes");
    const auto has_scope = [&](const std::string& scope_name) {
        return std::any_of(
            scopes.begin(),
            scopes.end(),
            [&](const json& scope) {
                return scope.at("scope_name").get<std::string>() == scope_name;
            });
    };

    EXPECT_TRUE(has_scope("costmap.grid_creation"));
    EXPECT_TRUE(has_scope("planner.goal_check"));
    EXPECT_TRUE(has_scope("planner.primitive_expansion_attempt"));

    for (const auto& scope : scopes) {
        EXPECT_GE(scope.at("count").get<uint64_t>(), 1u);
        EXPECT_GE(scope.at("total_ms").get<double>(), 0.0);
        EXPECT_GE(scope.at("avg_ms").get<double>(), 0.0);
        EXPECT_GE(scope.at("max_ms").get<double>(), 0.0);
    }
}

TEST(PlannerVisualizerServiceTest, DebugModeIncludesProfilingForEachRetryAttempt) {
    auto service = PlannerVisualizerService(PlannerVisualizerServiceConfig{
        makeTempConfigsRoot(true, true),
        "Pro_XD"
    });
    const auto map = service.loadMap(
        MapLoadRequest{"retry_map.yaml", retryManeuveringMapYaml(), "Pro_XD"});

    const auto result = service.plan(PlanRequest{
        map.map_id,
        "Pro_XD",
        PoseDto{0.15, 14.4, 0.0},
        PoseDto{4.7, 4.84, 0.0}
    });

    ASSERT_TRUE(result.success) << result.detail;
    ASSERT_GE(result.attempted_profiles.size(), 2u);
    EXPECT_EQ(result.selected_profile, "relaxed_profile");
    ASSERT_FALSE(result.debug_report_path.empty());
    ASSERT_TRUE(std::filesystem::exists(result.debug_report_path));

    const json report = loadJsonFile(result.debug_report_path);
    ASSERT_TRUE(report.contains("attempts"));
    ASSERT_GE(report.at("attempts").size(), 2u);

    bool saw_failed_attempt = false;
    bool saw_successful_attempt = false;
    for (const auto& attempt : report.at("attempts")) {
        ASSERT_TRUE(attempt.contains("profiling"));
        ASSERT_TRUE(attempt.at("profiling").contains("scopes"));
        ASSERT_FALSE(attempt.at("profiling").at("scopes").empty());
        if (attempt.at("result").at("success").get<bool>()) {
            saw_successful_attempt = true;
        } else {
            saw_failed_attempt = true;
        }
    }

    EXPECT_TRUE(saw_failed_attempt);
    EXPECT_TRUE(saw_successful_attempt);
}

TEST(PlannerVisualizerServiceTest, OppositeHeadingFailureIncludesTurningRadiusGuidance) {
    auto service = makeService();
    const auto map = service.loadMap(
        MapLoadRequest{"maneuvering_map.yaml", maneuveringMapYaml(), "Pro_XD"});

    const auto result = service.plan(PlanRequest{
        map.map_id,
        "Pro_XD",
        PoseDto{1.5, 3.0, 0.0},
        PoseDto{4.5, 3.0, 180.0}
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
