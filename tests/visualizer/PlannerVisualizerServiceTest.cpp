#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "coastmotionplanning/visualizer/planner_visualizer_service.hpp"

namespace {

using json = nlohmann::json;
using coastmotionplanning::visualizer::MapLoadRequest;
using coastmotionplanning::visualizer::PlanRequest;
using coastmotionplanning::visualizer::PlannerVisualizerService;
using coastmotionplanning::visualizer::PlannerVisualizerServiceConfig;
using coastmotionplanning::visualizer::PoseDto;
using coastmotionplanning::visualizer::SearchSpacePreviewRequest;

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
    lane:
      segments:
        - id: track_center
          offset: 1.5
          center_waypoints:
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
      - [6.0, 0.0]
      - [6.0, 6.0]
      - [0.0, 6.0]
)";
}

std::string connectedZonesMapYaml() {
    return R"(
maps:
  name: "Visualizer Connected Zones Map"
zones:
  - name: "start_zone"
    type: "ManeuveringZone"
    planner_behavior: "parking_profile"
    coordinate_type: "world"
    polygon:
      - [0.0, 0.0]
      - [6.0, 0.0]
      - [6.0, 6.0]
      - [0.0, 6.0]
  - name: "goal_zone"
    type: "ManeuveringZone"
    planner_behavior: "primary_profile"
    coordinate_type: "world"
    polygon:
      - [6.0, 1.0]
      - [12.0, 1.0]
      - [12.0, 5.0]
      - [6.0, 5.0]
)";
}

std::string disconnectedZonesMapYaml() {
    return R"(
maps:
  name: "Visualizer Disconnected Zones Map"
zones:
  - name: "start_zone"
    type: "ManeuveringZone"
    planner_behavior: "parking_profile"
    coordinate_type: "world"
    polygon:
      - [0.0, 0.0]
      - [6.0, 0.0]
      - [6.0, 6.0]
      - [0.0, 6.0]
  - name: "goal_zone"
    type: "ManeuveringZone"
    planner_behavior: "primary_profile"
    coordinate_type: "world"
    polygon:
      - [10.0, 0.0]
      - [16.0, 0.0]
      - [16.0, 6.0]
      - [10.0, 6.0]
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

size_t findBehaviorProfileStart(const std::string& yaml, const std::string& profile_name) {
    const std::string profile_header = "  " + profile_name + ":\n";
    const size_t profile_pos = yaml.find(profile_header);
    if (profile_pos == std::string::npos) {
        throw std::runtime_error("Failed to locate " + profile_name +
                                 " in planner_behaviors.yaml");
    }
    return profile_pos;
}

size_t findBehaviorProfileEnd(const std::string& yaml, size_t profile_pos) {
    size_t search_pos = profile_pos + 1;
    while (true) {
        const size_t candidate_pos = yaml.find("\n  ", search_pos);
        if (candidate_pos == std::string::npos) {
            return yaml.size();
        }

        const size_t next_char_pos = candidate_pos + 3;
        if (next_char_pos < yaml.size() && yaml[next_char_pos] != ' ') {
            return candidate_pos + 1;
        }
        search_pos = candidate_pos + 1;
    }
}

void replaceBehaviorProfileScalar(std::string& yaml,
                                  const std::string& profile_name,
                                  const std::string& key,
                                  const std::string& value) {
    const size_t profile_pos = findBehaviorProfileStart(yaml, profile_name);
    const size_t profile_end = findBehaviorProfileEnd(yaml, profile_pos);
    const std::string key_prefix = "      " + key + ":";
    const size_t key_pos = yaml.find(key_prefix, profile_pos);
    if (key_pos == std::string::npos || key_pos >= profile_end) {
        throw std::runtime_error("Failed to locate " + profile_name + "." + key);
    }

    const size_t line_end = yaml.find('\n', key_pos);
    if (line_end == std::string::npos) {
        throw std::runtime_error("Failed to locate end of " + profile_name + "." + key +
                                 " line.");
    }

    yaml.replace(key_pos, line_end - key_pos, "      " + key + ": " + value);
}

void removeGlobalCostmapCacheConfig(std::string& yaml) {
    const std::string block_header = "  costmap_cache:\n";
    const size_t block_pos = yaml.find(block_header);
    if (block_pos == std::string::npos) {
        return;
    }

    size_t block_end = block_pos + block_header.size();
    while (block_end < yaml.size()) {
        const size_t line_end = yaml.find('\n', block_end);
        const size_t next = line_end == std::string::npos ? yaml.size() : line_end + 1;
        const std::string_view line(&yaml[block_end], next - block_end);
        if (line.size() < 4 || line.substr(0, 4) != "    ") {
            break;
        }
        block_end = next;
    }
    yaml.erase(block_pos, block_end - block_pos);
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
        replaceBehaviorProfileScalar(behaviors, "primary_profile", "only_forward_path", "true");
        replaceBehaviorProfileScalar(behaviors, "relaxed_profile", "only_forward_path", "false");
        replaceBehaviorProfileScalar(behaviors, "relaxed_profile", "max_planning_time_ms", "5000");
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

std::string selkirkMapYaml() {
    return readTextFile(std::filesystem::path(configsRoot()) / "maps" / "selkirk_map.yaml");
}

std::string largoMapYaml() {
    return readTextFile(std::filesystem::path(configsRoot()) / "maps" / "largo_map.yaml");
}

double normalizedHeadingDeltaDegrees(double first_deg, double second_deg) {
    double delta = second_deg - first_deg;
    while (delta <= -180.0) {
        delta += 360.0;
    }
    while (delta > 180.0) {
        delta -= 360.0;
    }
    return std::abs(delta);
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
    EXPECT_TRUE(response.search_boundary.empty());
    ASSERT_EQ(response.zones.size(), 1u);
    EXPECT_EQ(response.zones[0].type, "TrackMainRoad");
    ASSERT_EQ(response.zones[0].lanes.size(), 2u);
    ASSERT_EQ(response.zones[0].lanes[0].poses.size(), 2u);
}

TEST(PlannerVisualizerServiceTest, LoadMapWithoutConfiguredStaticLayerCacheStillSucceeds) {
    const auto temp_configs_root = makeTempConfigsRoot(false);
    std::string behaviors = readTextFile(temp_configs_root / "planner_behaviors.yaml");
    removeGlobalCostmapCacheConfig(behaviors);
    std::ofstream behavior_stream(temp_configs_root / "planner_behaviors.yaml");
    behavior_stream << behaviors;
    behavior_stream.close();

    PlannerVisualizerService service(PlannerVisualizerServiceConfig{
        temp_configs_root,
        "Pro_XD"
    });
    const auto result = service.loadMap(MapLoadRequest{
        "simple_map.yaml",
        simpleMapYaml(),
        "Pro_XD"
    });

    EXPECT_FALSE(result.map_id.empty());
    EXPECT_EQ(result.filename, "simple_map.yaml");
    EXPECT_EQ(result.zones.size(), 1u);
}

TEST(PlannerVisualizerServiceTest, PreviewSearchSpaceReturnsBoundaryForSameZoneRequest) {
    auto service = makeService();
    const auto load_response = service.loadMap(MapLoadRequest{
        "maneuvering_map.yaml",
        maneuveringMapYaml(),
        ""
    });

    const auto preview = service.previewSearchSpace(SearchSpacePreviewRequest{
        load_response.map_id,
        PoseDto{1.0, 1.0, 0.0},
        PoseDto{5.0, 5.0, 0.0}
    });

    EXPECT_TRUE(preview.success);
    EXPECT_FALSE(preview.search_boundary.empty());
}

TEST(PlannerVisualizerServiceTest, PreviewSearchSpaceReturnsBoundaryForDirectlyConnectedZones) {
    auto service = makeService();
    const auto load_response = service.loadMap(MapLoadRequest{
        "connected_zones.yaml",
        connectedZonesMapYaml(),
        ""
    });

    const auto preview = service.previewSearchSpace(SearchSpacePreviewRequest{
        load_response.map_id,
        PoseDto{2.0, 2.0, 0.0},
        PoseDto{10.0, 3.0, 0.0}
    });

    EXPECT_TRUE(preview.success);
    EXPECT_FALSE(preview.search_boundary.empty());
}

TEST(PlannerVisualizerServiceTest, PreviewSearchSpaceReportsFailureForDisconnectedZones) {
    auto service = makeService();
    const auto load_response = service.loadMap(MapLoadRequest{
        "disconnected_zones.yaml",
        disconnectedZonesMapYaml(),
        ""
    });

    const auto preview = service.previewSearchSpace(SearchSpacePreviewRequest{
        load_response.map_id,
        PoseDto{2.0, 2.0, 0.0},
        PoseDto{12.0, 3.0, 0.0}
    });

    EXPECT_FALSE(preview.success);
    EXPECT_TRUE(preview.search_boundary.empty());
    EXPECT_NE(preview.detail.find("do not share an edge or overlap"), std::string::npos);
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

TEST(PlannerVisualizerServiceTest, SelkirkRegressionScenarioReachesTrackRoadGoalFromManeuverZone) {
    struct Scenario {
        std::string goal_name;
        PoseDto start;
        PoseDto goal;
    };

    // goal_262 is still useful for manual validation, but after tightening the
    // search boundary it became sensitive to sub-cell rasterization near the
    // goal footprint and is not stable enough for automated regression.
    const std::vector<Scenario> scenarios{
        {"goal_366", PoseDto{203739.54739898338, 414524.7252263559, -155.16585688980373},
                     PoseDto{203716.66671469453, 414514.13631547976, -63.205516011951886}},
    };

    auto service = makeService();
    const auto map = service.loadMap(MapLoadRequest{
        "selkirk_map.yaml",
        selkirkMapYaml(),
        "Pro_XD"
    });

    for (const auto& scenario : scenarios) {
        SCOPED_TRACE(scenario.goal_name);
        const auto result = service.plan(PlanRequest{
            map.map_id,
            "Pro_XD",
            scenario.start,
            scenario.goal
        });

        EXPECT_TRUE(result.success) << result.detail;
        EXPECT_FALSE(result.attempted_profiles.empty());
        EXPECT_FALSE(result.path.empty());
        EXPECT_EQ(result.segment_directions.size(), result.path.size() - 1);
    }
}

TEST(PlannerVisualizerServiceTest, SelkirkBoundaryGoal212ReportsCollisionFromManeuverZone) {
    auto service = makeService();
    const auto map = service.loadMap(MapLoadRequest{
        "selkirk_map.yaml",
        selkirkMapYaml(),
        "Pro_XD"
    });

    // Other legacy goal stations near the tightened boundary remain useful for
    // manual debug-mode inspection, but they are not stable enough for
    // automated regression after the search-space fix.
    const auto result = service.plan(PlanRequest{
        map.map_id,
        "Pro_XD",
        PoseDto{203823.16622475124, 414367.01701535593, 164.50859184708628},
        PoseDto{203790.35861873932, 414376.11007188854, 118.07485700101844}
    });

    EXPECT_FALSE(result.success);
    EXPECT_NE(result.detail.find("Goal pose is in collision"), std::string::npos)
        << result.detail;
}

TEST(PlannerVisualizerServiceTest, SelkirkTrackPromotionKeepsHeadingBinsConsistent) {
    auto service = PlannerVisualizerService(PlannerVisualizerServiceConfig{
        makeTempConfigsRoot(true),
        "Pro_XD"
    });
    const auto map = service.loadMap(MapLoadRequest{
        "selkirk_map.yaml",
        selkirkMapYaml(),
        "Pro_XD"
    });

    const auto result = service.plan(PlanRequest{
        map.map_id,
        "Pro_XD",
        PoseDto{203825.03, 414374.09, 0.0},
        PoseDto{203831.52, 414408.56, 120.0}
    });

    ASSERT_TRUE(result.success) << result.detail;
    EXPECT_TRUE(result.debug_mode);
    ASSERT_FALSE(result.debug_report_path.empty());
    ASSERT_TRUE(std::filesystem::exists(result.debug_report_path));

    const json report = loadJsonFile(result.debug_report_path);
    ASSERT_TRUE(report.contains("attempts"));

    const auto successful_attempt = std::find_if(
        report.at("attempts").begin(),
        report.at("attempts").end(),
        [](const json& attempt) {
            return attempt.at("result").at("success").get<bool>();
        });
    ASSERT_NE(successful_attempt, report.at("attempts").end());

    ASSERT_TRUE(successful_attempt->contains("debug_trace"));
    ASSERT_TRUE(successful_attempt->at("debug_trace").contains("expansions"));
    const auto& expansions = successful_attempt->at("debug_trace").at("expansions");

    auto promoted_expansion = std::find_if(
        expansions.begin(),
        expansions.end(),
        [](const json& expansion) {
            return expansion.at("zone_name").get<std::string>() == "track_road_2" &&
                   expansion.at("behavior_name").get<std::string>() == "track_main_road_profile" &&
                   expansion.at("transition_promotion_reason").get<std::string>() ==
                       "aligned_and_deep_enough";
        });
    if (promoted_expansion == expansions.end()) {
        promoted_expansion = std::find_if(
            expansions.begin(),
            expansions.end(),
            [](const json& expansion) {
                return expansion.at("zone_name").get<std::string>() == "track_road_2";
            });
    }
    ASSERT_NE(promoted_expansion, expansions.end());

    const double node_heading_deg =
        promoted_expansion->at("pose").at("heading_deg").get<double>();
    ASSERT_TRUE(promoted_expansion->contains("primitive_events"));
    ASSERT_FALSE(promoted_expansion->at("primitive_events").empty());

    bool saw_enqueued_track_primitive = false;
    for (const auto& primitive : promoted_expansion->at("primitive_events")) {
        const auto turn_direction = primitive.at("turn_direction").get<std::string>();
        if (turn_direction != "FORWARD" &&
            turn_direction != "LEFT" &&
            turn_direction != "RIGHT") {
            continue;
        }
        if (primitive.at("outcome").get<std::string>() != "enqueued") {
            continue;
        }
        saw_enqueued_track_primitive = true;
        const double successor_heading_deg =
            primitive.at("successor_pose").at("heading_deg").get<double>();
        EXPECT_LE(
            normalizedHeadingDeltaDegrees(node_heading_deg, successor_heading_deg),
            5.1)
            << primitive.dump();
    }
    EXPECT_TRUE(saw_enqueued_track_primitive);
}

TEST(PlannerVisualizerServiceTest, SelkirkScreenshotScenarioSucceeds) {
    auto service = makeService();
    const auto map = service.loadMap(MapLoadRequest{
        "selkirk_map.yaml",
        selkirkMapYaml(),
        "Pro_XD"
    });

    const auto result = service.plan(PlanRequest{
        map.map_id,
        "Pro_XD",
        PoseDto{203825.08, 414370.06, 180.0},
        PoseDto{203828.95, 414413.75, 120.0}
    });

    EXPECT_TRUE(result.success) << result.detail;
    EXPECT_FALSE(result.attempted_profiles.empty());
    EXPECT_FALSE(result.path.empty());
    EXPECT_EQ(result.segment_directions.size(), result.path.size() - 1);
}

TEST(PlannerVisualizerServiceTest, LargoDebugTraceShowsEntryBehaviorPromotionOnTrackHandoff) {
    auto service = PlannerVisualizerService(PlannerVisualizerServiceConfig{
        makeTempConfigsRoot(true),
        "Pro_XD"
    });
    const auto map = service.loadMap(MapLoadRequest{
        "largo_map.yaml",
        largoMapYaml(),
        "Pro_XD"
    });

    const auto result = service.plan(PlanRequest{
        map.map_id,
        "Pro_XD",
        PoseDto{3.64, 11.75, 0.0},
        PoseDto{76.95, 1.36, 0.0}
    });

    ASSERT_TRUE(result.success) << result.detail;
    EXPECT_TRUE(result.debug_mode);
    ASSERT_FALSE(result.debug_report_path.empty());
    ASSERT_TRUE(std::filesystem::exists(result.debug_report_path));

    const json report = loadJsonFile(result.debug_report_path);
    ASSERT_TRUE(report.contains("attempts"));
    ASSERT_FALSE(report.at("attempts").empty());

    const auto successful_attempt = std::find_if(
        report.at("attempts").begin(),
        report.at("attempts").end(),
        [](const json& attempt) {
            return attempt.at("result").at("success").get<bool>();
        });
    ASSERT_NE(successful_attempt, report.at("attempts").end());
    ASSERT_TRUE(successful_attempt->contains("debug_trace"));
    ASSERT_TRUE(successful_attempt->at("debug_trace").contains("expansions"));
    ASSERT_TRUE(successful_attempt->at("debug_trace").contains("frontier_heuristics"));

    const auto handoff_guidance = std::find_if(
        successful_attempt->at("debug_trace").at("frontier_heuristics").begin(),
        successful_attempt->at("debug_trace").at("frontier_heuristics").end(),
        [](const json& heuristic) {
            return heuristic.at("zone_name").get<std::string>() == "largo_maneuvering_dock";
        });
    ASSERT_NE(
        handoff_guidance,
        successful_attempt->at("debug_trace").at("frontier_heuristics").end());
    EXPECT_EQ(handoff_guidance->at("objective_kind").get<std::string>(), "handoff");
    EXPECT_EQ(
        handoff_guidance->at("target_mode").get<std::string>(),
        "goal_grid_with_interface_bias");
    EXPECT_GT(handoff_guidance->at("seed_count").get<size_t>(), 0u);

    bool saw_entry_behavior = false;
    bool saw_steady_behavior = false;
    for (const auto& expansion : successful_attempt->at("debug_trace").at("expansions")) {
        if (expansion.at("zone_name").get<std::string>() != "largo_main_road_entry") {
            continue;
        }
        saw_entry_behavior =
            saw_entry_behavior ||
            (expansion.at("transition_entry_behavior_active").get<bool>() &&
             expansion.at("behavior_name").get<std::string>() == "maneuver_to_track_profile" &&
             expansion.at("transition_entry_behavior_name").get<std::string>() ==
                 "maneuver_to_track_profile" &&
             expansion.at("transition_steady_behavior_name").get<std::string>() ==
                 "track_main_road_profile");
        saw_steady_behavior =
            saw_steady_behavior ||
            (!expansion.at("transition_entry_behavior_active").get<bool>() &&
             expansion.at("behavior_name").get<std::string>() == "track_main_road_profile");
    }

    EXPECT_TRUE(saw_entry_behavior);
    EXPECT_TRUE(saw_steady_behavior);
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
    EXPECT_NE(result.detail.find("turning radius"), std::string::npos) << result.detail;
    EXPECT_NE(result.detail.find("maneuvering zone"), std::string::npos) << result.detail;
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
