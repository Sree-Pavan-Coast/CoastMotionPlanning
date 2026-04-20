#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "coastmotionplanning/math/angle.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/motion_primitives/car_motion_table.hpp"
#include "coastmotionplanning/motion_primitives/motion_primitive_types.hpp"
#include "coastmotionplanning/config/robots_parser.hpp"
#include "coastmotionplanning/planning/hybrid_a_star_planner.hpp"
#include "coastmotionplanning/planning/planner_behavior_set.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

namespace {

using coastmotionplanning::geometry::Polygon2d;
using coastmotionplanning::math::Angle;
using coastmotionplanning::math::Pose2d;
using coastmotionplanning::planning::HybridAStarPlanner;
using coastmotionplanning::planning::HybridAStarPlannerRequest;
using coastmotionplanning::planning::HybridAStarPlannerResult;
using coastmotionplanning::planning::PlannerBehaviorProfile;
using coastmotionplanning::planning::PlannerBehaviorSet;
using coastmotionplanning::planning::ResolvedPlannerBehavior;
using coastmotionplanning::robot::Car;

Polygon2d makeRectangle(double min_x, double min_y, double max_x, double max_y) {
    Polygon2d polygon;
    polygon.outer().push_back({min_x, min_y});
    polygon.outer().push_back({max_x, min_y});
    polygon.outer().push_back({max_x, max_y});
    polygon.outer().push_back({min_x, max_y});
    polygon.outer().push_back({min_x, min_y});
    return polygon;
}

std::string repoPath(const std::string& relative_path) {
    return std::string(COAST_REPO_ROOT) + "/" + relative_path;
}

PlannerBehaviorSet applyRobotConstraints(PlannerBehaviorSet behavior_set,
                                         const std::string& robot_name = "Pro_XD") {
    const auto car_definition = coastmotionplanning::config::RobotsParser::loadCarDefinition(
        repoPath("configs/robots.yaml"),
        robot_name);
    behavior_set.overrideMotionPrimitiveConstraints(
        car_definition.minTurningRadiusMeters(),
        car_definition.maxSteerAngleRadians());
    return behavior_set;
}

PlannerBehaviorSet loadBehaviorSet() {
    return applyRobotConstraints(
        PlannerBehaviorSet::loadFromFile(repoPath("configs/planner_behaviors.yaml")));
}

std::string readFile(const std::string& path) {
    std::ifstream stream(path);
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
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
        throw std::runtime_error("Failed to locate " + profile_name + "." + key +
                                 " in planner_behaviors.yaml");
    }

    const size_t line_end = yaml.find('\n', key_pos);
    if (line_end == std::string::npos) {
        throw std::runtime_error("Failed to locate end of " + profile_name + "." + key +
                                 " line.");
    }

    yaml.replace(key_pos, line_end - key_pos, "      " + key + ": " + value);
}

PlannerBehaviorSet loadCustomPrimaryProfileBehaviorSet(bool only_forward_path,
                                                       double min_same_motion_length_m,
                                                       const std::string& robot_name = "E_Transit",
                                                       bool debug_mode = false,
                                                       double min_goal_straight_approach_m = 0.0,
                                                       int max_planning_time_ms = -1) {
    const std::filesystem::path temp_dir =
        std::filesystem::temp_directory_path() / "coastmotionplanning_hybrid_astar_test";
    std::filesystem::create_directories(temp_dir);

    std::string behaviors = readFile(repoPath("configs/planner_behaviors.yaml"));
    const std::string debug_mode_disabled = "  debug_mode: false";
    const size_t debug_mode_pos = behaviors.find(debug_mode_disabled);
    if (debug_mode && debug_mode_pos == std::string::npos) {
        throw std::runtime_error(
            "Failed to locate global.debug_mode in planner_behaviors.yaml");
    }
    if (debug_mode) {
        behaviors.replace(
            debug_mode_pos,
            debug_mode_disabled.size(),
            "  debug_mode: true");
    }
    replaceBehaviorProfileScalar(
        behaviors,
        "primary_profile",
        "only_forward_path",
        only_forward_path ? "true" : "false");
    replaceBehaviorProfileScalar(
        behaviors,
        "primary_profile",
        "min_path_len_in_same_motion",
        std::to_string(min_same_motion_length_m));
    replaceBehaviorProfileScalar(
        behaviors,
        "primary_profile",
        "min_goal_straight_approach_m",
        std::to_string(min_goal_straight_approach_m));
    if (max_planning_time_ms > 0) {
        replaceBehaviorProfileScalar(
            behaviors,
            "primary_profile",
            "max_planning_time_ms",
            std::to_string(max_planning_time_ms));
    }

    std::ofstream behavior_file(temp_dir / "planner_behaviors.yaml");
    behavior_file << behaviors;
    behavior_file.close();

    return applyRobotConstraints(
        PlannerBehaviorSet::loadFromFile((temp_dir / "planner_behaviors.yaml").string()),
        robot_name);
}

PlannerBehaviorSet loadBehaviorSetWithoutAnalyticExpansion(
    const std::vector<std::string>& profile_names,
    const std::string& robot_name = "Pro_XD") {
    const std::filesystem::path temp_dir =
        std::filesystem::temp_directory_path() / "coastmotionplanning_hybrid_astar_test";
    std::filesystem::create_directories(temp_dir);

    std::string behaviors = readFile(repoPath("configs/planner_behaviors.yaml"));
    for (const auto& profile_name : profile_names) {
        replaceBehaviorProfileScalar(behaviors, profile_name, "analytic_shot", "false");
        replaceBehaviorProfileScalar(
            behaviors,
            profile_name,
            "near_goal_analytic_expansion",
            "false");
    }

    std::ofstream behavior_file(temp_dir / "planner_behaviors.yaml");
    behavior_file << behaviors;
    behavior_file.close();

    return applyRobotConstraints(
        PlannerBehaviorSet::loadFromFile((temp_dir / "planner_behaviors.yaml").string()),
        robot_name);
}

PlannerBehaviorSet loadCustomTrackRoadProfileBehaviorSet(
    double min_goal_straight_approach_m,
    bool disable_analytic_expansion,
    const std::string& robot_name = "Pro_XD") {
    const std::filesystem::path temp_dir =
        std::filesystem::temp_directory_path() / "coastmotionplanning_hybrid_astar_test";
    std::filesystem::create_directories(temp_dir);

    std::string behaviors = readFile(repoPath("configs/planner_behaviors.yaml"));
    replaceBehaviorProfileScalar(
        behaviors,
        "track_main_road_profile",
        "min_goal_straight_approach_m",
        std::to_string(min_goal_straight_approach_m));
    replaceBehaviorProfileScalar(
        behaviors,
        "track_main_road_profile",
        "max_planning_time_ms",
        "5000");
    replaceBehaviorProfileScalar(
        behaviors,
        "track_main_road_profile",
        "only_forward_path",
        "false");
    if (disable_analytic_expansion) {
        replaceBehaviorProfileScalar(
            behaviors,
            "track_main_road_profile",
            "analytic_shot",
            "false");
        replaceBehaviorProfileScalar(
            behaviors,
            "track_main_road_profile",
            "near_goal_analytic_expansion",
            "false");
    }

    std::ofstream behavior_file(temp_dir / "planner_behaviors.yaml");
    behavior_file << behaviors;
    behavior_file.close();

    return applyRobotConstraints(
        PlannerBehaviorSet::loadFromFile((temp_dir / "planner_behaviors.yaml").string()),
        robot_name);
}

double straightPrimitiveLengthM(const PlannerBehaviorProfile& profile) {
    coastmotionplanning::motion_primitives::MotionTableConfig motion_config;
    motion_config.minimum_turning_radius = static_cast<float>(
        profile.motion_primitives.min_turning_radius_m /
        profile.planner.xy_grid_resolution_m);
    motion_config.num_angle_quantization = static_cast<unsigned int>(
        profile.motion_primitives.num_angle_bins);

    coastmotionplanning::motion_primitives::CarMotionTable motion_table;
    motion_table.initReedsShepp(motion_config);
    return static_cast<double>(motion_table.getTravelCost(0)) *
           profile.planner.xy_grid_resolution_m;
}

std::shared_ptr<coastmotionplanning::zones::ManeuveringZone> makeManeuveringZone(
    double min_x,
    double min_y,
    double max_x,
    double max_y,
    const std::string& behavior = "") {
    auto zone = std::make_shared<coastmotionplanning::zones::ManeuveringZone>(
        makeRectangle(min_x, min_y, max_x, max_y), "maneuvering");
    if (!behavior.empty()) {
        zone->setPlannerBehavior(behavior);
    }
    return zone;
}

std::shared_ptr<coastmotionplanning::zones::TrackMainRoad> makeTrackZone(
    double min_x,
    double min_y,
    double max_x,
    double max_y,
    const std::string& behavior = "") {
    auto zone = std::make_shared<coastmotionplanning::zones::TrackMainRoad>(
        makeRectangle(min_x, min_y, max_x, max_y),
        std::vector<coastmotionplanning::geometry::Point2d>{
            {min_x + 1.0, min_y + ((max_y - min_y) * 0.5)},
            {max_x - 1.0, min_y + ((max_y - min_y) * 0.5)}
        },
        std::vector<double>{
            (max_y - min_y) * 0.25,
            (max_y - min_y) * 0.25
        },
        "track");
    if (!behavior.empty()) {
        zone->setPlannerBehavior(behavior);
    }
    return zone;
}

Pose2d makePose(double x, double y, double yaw_deg = 0.0) {
    return Pose2d{x, y, Angle::from_degrees(yaw_deg)};
}

Car makeCar() {
    return Car(2.0, 3.0, 0.5, 0.5);
}

std::vector<double> accumulateRunLengths(
    const std::vector<Pose2d>& poses,
    const std::vector<coastmotionplanning::common::MotionDirection>& segment_directions) {
    std::vector<double> run_lengths;
    if (poses.size() < 2 || segment_directions.empty()) {
        return run_lengths;
    }

    coastmotionplanning::common::MotionDirection current_direction = segment_directions.front();
    double current_length = 0.0;
    for (size_t i = 0; i < segment_directions.size(); ++i) {
        const double segment_length = std::hypot(
            poses[i + 1].x - poses[i].x,
            poses[i + 1].y - poses[i].y);
        if (segment_directions[i] != current_direction) {
            run_lengths.push_back(current_length);
            current_direction = segment_directions[i];
            current_length = segment_length;
        } else {
            current_length += segment_length;
        }
    }

    run_lengths.push_back(current_length);
    return run_lengths;
}

TEST(HybridAStarPlannerPolicyTest, TrackRoadWithoutForwardOnlyProfileAllowsReverse) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const auto track_zone = makeTrackZone(0.0, -4.0, 10.0, 4.0);
    const ResolvedPlannerBehavior resolved{
        0,
        track_zone,
        "parking_profile",
        &behavior_set.get("parking_profile"),
        false,
        false
    };

    EXPECT_EQ(
        HybridAStarPlanner::selectHeuristicModel(resolved),
        coastmotionplanning::costs::HeuristicModel::REEDS_SHEPP);
    EXPECT_TRUE(HybridAStarPlanner::isPrimitiveAllowed(
        coastmotionplanning::motion_primitives::TurnDirection::FORWARD, resolved));
    EXPECT_TRUE(HybridAStarPlanner::isPrimitiveAllowed(
        coastmotionplanning::motion_primitives::TurnDirection::REVERSE, resolved));
    EXPECT_TRUE(HybridAStarPlanner::isPrimitiveAllowed(
        coastmotionplanning::motion_primitives::TurnDirection::REV_LEFT, resolved));
}

TEST(HybridAStarPlannerPolicyTest, ReverseAllowedZonesUseReedsSheppAndAllowReverse) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const auto maneuvering_zone = makeManeuveringZone(0.0, -4.0, 10.0, 4.0);
    const ResolvedPlannerBehavior resolved{
        0,
        maneuvering_zone,
        "parking_profile",
        &behavior_set.get("parking_profile"),
        false,
        false
    };

    EXPECT_EQ(
        HybridAStarPlanner::selectHeuristicModel(resolved),
        coastmotionplanning::costs::HeuristicModel::REEDS_SHEPP);
    EXPECT_TRUE(HybridAStarPlanner::isPrimitiveAllowed(
        coastmotionplanning::motion_primitives::TurnDirection::REVERSE, resolved));
    EXPECT_TRUE(HybridAStarPlanner::isPrimitiveAllowed(
        coastmotionplanning::motion_primitives::TurnDirection::REV_RIGHT, resolved));
}

TEST(HybridAStarPlannerPolicyTest, OnlyForwardProfilePrunesReverseEvenInManeuveringZone) {
    const auto maneuvering_zone = makeManeuveringZone(0.0, -4.0, 10.0, 4.0);
    PlannerBehaviorProfile forward_only_profile;
    forward_only_profile.planner.only_forward_path = true;

    const ResolvedPlannerBehavior resolved{
        0,
        maneuvering_zone,
        "custom_forward_only",
        &forward_only_profile,
        false,
        false
    };

    EXPECT_EQ(
        HybridAStarPlanner::selectHeuristicModel(resolved),
        coastmotionplanning::costs::HeuristicModel::DUBINS);
    EXPECT_TRUE(HybridAStarPlanner::isPrimitiveAllowed(
        coastmotionplanning::motion_primitives::TurnDirection::FORWARD, resolved));
    EXPECT_FALSE(HybridAStarPlanner::isPrimitiveAllowed(
        coastmotionplanning::motion_primitives::TurnDirection::REVERSE, resolved));
    EXPECT_FALSE(HybridAStarPlanner::isPrimitiveAllowed(
        coastmotionplanning::motion_primitives::TurnDirection::REV_LEFT, resolved));
}

TEST(HybridAStarPlannerTest, SameZoneCarPlanningSucceeds) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 12.0, 4.0)
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(2.0, 0.0);
    request.goal = makePose(7.0, 0.0);
    request.initial_behavior_name = "primary_profile";

    const auto result = planner.plan(request);

    ASSERT_TRUE(result.success) << result.detail;
    ASSERT_FALSE(result.poses.empty());
    EXPECT_EQ(result.behavior_sequence.front(), "primary_profile");
    EXPECT_EQ(result.behavior_sequence.back(), "primary_profile");
    EXPECT_EQ(result.segment_directions.size(), result.poses.size() - 1);
}

TEST(HybridAStarPlannerTest, TrackRoadProfileFollowsForwardLaneTowardGoalStation) {
    const PlannerBehaviorSet behavior_set =
        loadBehaviorSetWithoutAnalyticExpansion({"track_main_road_profile"});
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -6.0, 24.0, 6.0, "track_main_road_profile")
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(3.0, -3.0, 0.0);
    request.goal = makePose(18.0, -3.0, 0.0);
    request.initial_behavior_name = "track_main_road_profile";

    const auto result = planner.plan(request);

    ASSERT_TRUE(result.success) << result.detail;
    ASSERT_GE(result.poses.size(), 2u);
    const double mean_y = std::accumulate(
        result.poses.begin(),
        result.poses.end(),
        0.0,
        [](double sum, const Pose2d& pose) { return sum + pose.y; }) /
        static_cast<double>(result.poses.size());
    EXPECT_LT(mean_y, -1.0);
}

TEST(HybridAStarPlannerTest, TrackRoadGoalStraightApproachCanRelaxLaneBiasNearGoal) {
    const PlannerBehaviorSet behavior_set =
        loadCustomTrackRoadProfileBehaviorSet(1.0, false);
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -6.0, 24.0, 6.0, "track_main_road_profile")
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(3.0, -3.0, 0.0);
    request.goal = makePose(18.0, -1.0, 0.0);
    request.initial_behavior_name = "track_main_road_profile";

    const auto result = planner.plan(request);

    ASSERT_TRUE(result.success) << result.detail;
    ASSERT_GE(result.poses.size(), 3u);
    const bool used_lane_before_goal = std::any_of(
        result.poses.begin() + 1,
        result.poses.end() - 1,
        [](const Pose2d& pose) { return pose.y < -2.5; });
    EXPECT_TRUE(used_lane_before_goal);
    EXPECT_NEAR(result.poses.back().y, -1.0, 0.25);
}

TEST(HybridAStarPlannerTest, DisconnectedCrossZonePlanningFailsBeforeSearch) {
    auto behavior_set = loadBehaviorSetWithoutAnalyticExpansion(
        {"relaxed_profile", "primary_profile", "parking_profile"});
    behavior_set.setMinimumPlanningTimeMs(1500);
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 10.0, 4.0),
        makeManeuveringZone(12.0, -4.0, 22.0, 4.0, "parking_profile")
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(2.0, 0.0);
    request.goal = makePose(17.0, 0.0);
    request.initial_behavior_name = "relaxed_profile";

    const auto result = planner.plan(request);

    EXPECT_FALSE(result.success);
    EXPECT_NE(result.detail.find("do not share an edge or overlap"), std::string::npos)
        << result.detail;
}

TEST(HybridAStarPlannerTest, MissingInitialBehaviorFailsFast) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 12.0, 4.0)
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(2.0, 0.0);
    request.goal = makePose(7.0, 0.0);
    request.initial_behavior_name = "missing_profile";

    const auto result = planner.plan(request);

    EXPECT_FALSE(result.success);
    EXPECT_NE(result.detail.find("missing_profile"), std::string::npos);
}

TEST(HybridAStarPlannerTest, StartOutsideAllZonesFailsBeforeSearch) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 12.0, 4.0)
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(-2.0, 0.0);
    request.goal = makePose(7.0, 0.0);
    request.initial_behavior_name = "primary_profile";

    const auto result = planner.plan(request);

    EXPECT_FALSE(result.success);
    EXPECT_NE(result.detail.find("not inside any zone"), std::string::npos);
}

TEST(HybridAStarPlannerTest, StartCollisionFailsOnSelectedZoneCostmap) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 12.0, 4.0)
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(0.2, 0.0);
    request.goal = makePose(7.0, 0.0);
    request.initial_behavior_name = "primary_profile";

    const auto result = planner.plan(request);

    EXPECT_FALSE(result.success);
    EXPECT_NE(result.detail.find("Start pose is in collision"), std::string::npos);
}

TEST(HybridAStarPlannerTest, RuntimeObstaclePolygonsTriggerPlannerCollisionValidation) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 12.0, 4.0)
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(2.0, 0.0);
    request.goal = makePose(7.0, 0.0);
    request.obstacle_polygons.push_back(makeRectangle(1.0, -1.5, 3.0, 1.5));
    request.initial_behavior_name = "primary_profile";

    const auto result = planner.plan(request);

    EXPECT_FALSE(result.success);
    EXPECT_NE(result.detail.find("Start pose is in collision"), std::string::npos);
}

TEST(HybridAStarPlannerTest, GoalMustAllowMinimumSameMotionLengthBeforeStopping) {
    const PlannerBehaviorSet behavior_set =
        loadCustomPrimaryProfileBehaviorSet(true, 5.0);
    const auto& primary_profile = behavior_set.get("primary_profile");
    const double straight_step_m = straightPrimitiveLengthM(primary_profile);
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 12.0, 4.0)
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(2.0, 0.0, 0.0);
    request.initial_behavior_name = "primary_profile";

    int first_success_steps = -1;
    HybridAStarPlannerResult long_enough_result;
    for (int steps = 1; steps <= 20; ++steps) {
        request.goal = makePose(2.0 + (steps * straight_step_m), 0.0, 0.0);
        const auto candidate_result = planner.plan(request);
        if (candidate_result.success) {
            first_success_steps = steps;
            long_enough_result = candidate_result;
            break;
        }
    }

    ASSERT_NE(first_success_steps, -1);
    EXPECT_GE(
        (first_success_steps * straight_step_m) + 1e-6,
        primary_profile.planner.min_path_len_in_same_motion);

    request.goal = makePose(2.0 + ((first_success_steps - 1) * straight_step_m), 0.0, 0.0);
    const auto too_short_result = planner.plan(request);
    EXPECT_FALSE(too_short_result.success);
    EXPECT_NE(
        too_short_result.detail.find("timed out") == std::string::npos
            ? too_short_result.detail.find("exhausted")
            : too_short_result.detail.find("timed out"),
        std::string::npos);

    ASSERT_TRUE(long_enough_result.success) << long_enough_result.detail;
    ASSERT_FALSE(long_enough_result.segment_directions.empty());
    EXPECT_TRUE(std::all_of(
        long_enough_result.segment_directions.begin(),
        long_enough_result.segment_directions.end(),
        [](coastmotionplanning::common::MotionDirection direction) {
            return direction == coastmotionplanning::common::MotionDirection::Forward;
        }));
}

TEST(HybridAStarPlannerTest, GoalStraightApproachRequirementPublishesImmediatelyWhenMet) {
    const PlannerBehaviorSet behavior_set =
        loadCustomPrimaryProfileBehaviorSet(false, 1.0, "Pro_XD", false, 2.0);
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 12.0, 4.0)
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(2.0, 0.0, 0.0);
    request.goal = makePose(7.0, 0.0, 0.0);
    request.initial_behavior_name = "primary_profile";

    const auto result = planner.plan(request);

    ASSERT_TRUE(result.success) << result.detail;
    EXPECT_EQ(result.segment_directions.size(), result.poses.size() - 1);
    EXPECT_EQ(
        result.detail.find("Returned best goal candidate after"),
        std::string::npos);
}

TEST(HybridAStarPlannerTest, ReturnsBestGoalCandidateWhenGoalStraightApproachIsUnmet) {
    const PlannerBehaviorSet behavior_set =
        loadCustomPrimaryProfileBehaviorSet(false, 1.0, "Pro_XD", false, 50.0, 200);
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 12.0, 4.0)
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(2.0, 0.0, 0.0);
    request.goal = makePose(7.0, 0.0, 0.0);
    request.initial_behavior_name = "primary_profile";

    const auto result = planner.plan(request);

    ASSERT_TRUE(result.success) << result.detail;
    EXPECT_NE(
        result.detail.find("Returned best goal candidate after"),
        std::string::npos);
    EXPECT_NE(
        result.detail.find("planner.min_goal_straight_approach_m=50"),
        std::string::npos);
}

TEST(HybridAStarPlannerTest, AnalyticExpansionOutputHonorsMinimumSameMotionLength) {
    auto behavior_set =
        loadCustomPrimaryProfileBehaviorSet(false, 5.0, "Pro_XD");
    behavior_set.setMinimumPlanningTimeMs(5000);
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeManeuveringZone(-5.0, -5.0, 15.0, 20.0, "primary_profile")
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(0.15, 14.4, 0.0);
    request.goal = makePose(4.7, 4.84, 0.0);
    request.initial_behavior_name = "primary_profile";

    const auto result = planner.plan(request);

    ASSERT_TRUE(result.success) << result.detail;
    EXPECT_NE(result.detail.find("analytic expansion"), std::string::npos);

    const auto run_lengths = accumulateRunLengths(result.poses, result.segment_directions);
    ASSERT_FALSE(run_lengths.empty());
    for (const double run_length : run_lengths) {
        EXPECT_GE(run_length + 1e-6, 5.0);
    }
}

TEST(HybridAStarPlannerTest, DebugModeCollectsProfilingScopesOnSuccess) {
    const PlannerBehaviorSet behavior_set =
        loadCustomPrimaryProfileBehaviorSet(false, 1.0, "Pro_XD", true);
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 12.0, 4.0)
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(2.0, 0.0);
    request.goal = makePose(7.0, 0.0);
    request.initial_behavior_name = "primary_profile";

    const auto result = planner.plan(request);

    ASSERT_TRUE(result.success) << result.detail;
    ASSERT_NE(result.debug_trace, nullptr);
    EXPECT_FALSE(result.debug_trace->profiling_scopes.empty());
    EXPECT_GE(result.debug_trace->total_planning_ms, 0.0);

    const auto has_scope = [&](const std::string& scope_name) {
        return std::any_of(
            result.debug_trace->profiling_scopes.begin(),
            result.debug_trace->profiling_scopes.end(),
            [&](const auto& scope) { return scope.scope_name == scope_name; });
    };

    EXPECT_TRUE(has_scope("costmap.grid_creation"));
    EXPECT_TRUE(has_scope("planner.goal_check"));
    EXPECT_TRUE(has_scope("planner.primitive_expansion_attempt"));
}

} // namespace
