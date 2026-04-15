#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
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

void replaceProfilePlannerLine(std::string& behaviors,
                               const std::string& profile_name,
                               const std::string& key,
                               const std::string& value) {
    const size_t profile_pos = behaviors.find("  " + profile_name + ":\n");
    if (profile_pos == std::string::npos) {
        throw std::runtime_error(
            "Failed to locate " + profile_name + " in planner_behaviors.yaml");
    }

    const std::string line_prefix = "      " + key + ":";
    const size_t line_pos = behaviors.find(line_prefix, profile_pos);
    if (line_pos == std::string::npos) {
        throw std::runtime_error(
            "Failed to locate " + profile_name + "." + key + " in planner_behaviors.yaml");
    }

    const size_t line_end = behaviors.find('\n', line_pos);
    if (line_end == std::string::npos) {
        throw std::runtime_error(
            "Failed to locate end of " + profile_name + "." + key + " line.");
    }

    behaviors.replace(
        line_pos,
        line_end - line_pos,
        line_prefix + " " + value);
}

PlannerBehaviorSet loadCustomPrimaryProfileBehaviorSet(bool only_forward_path,
                                                       double min_same_motion_length_m,
                                                       const std::string& robot_name = "E_Transit",
                                                       bool debug_mode = false,
                                                       double goal_approach_straight_distance_m = 2.0,
                                                       bool analytic_shot = true,
                                                       bool near_goal_analytic_expansion = true) {
    const std::filesystem::path temp_dir =
        std::filesystem::temp_directory_path() / "coastmotionplanning_hybrid_astar_test";
    std::filesystem::create_directories(temp_dir);

    std::ofstream master_file(temp_dir / "master_params.yaml");
    master_file << readFile(repoPath("configs/master_params.yaml"));
    master_file.close();

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
    replaceProfilePlannerLine(
        behaviors,
        "primary_profile",
        "only_forward_path",
        only_forward_path ? "true" : "false");
    replaceProfilePlannerLine(
        behaviors,
        "primary_profile",
        "min_path_len_in_same_motion",
        std::to_string(min_same_motion_length_m));
    replaceProfilePlannerLine(
        behaviors,
        "primary_profile",
        "goal_approach_straight_distance_m",
        std::to_string(goal_approach_straight_distance_m));
    replaceProfilePlannerLine(
        behaviors,
        "primary_profile",
        "analytic_shot",
        analytic_shot ? "true" : "false");
    replaceProfilePlannerLine(
        behaviors,
        "primary_profile",
        "near_goal_analytic_expansion",
        near_goal_analytic_expansion ? "true" : "false");

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

double normalizeAngleSigned(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle - kPi;
}

bool tailMaintainsHeading(const std::vector<Pose2d>& poses,
                          double tail_distance_m,
                          double heading_tolerance_rad) {
    if (poses.size() < 2) {
        return false;
    }

    double remaining_distance_m = tail_distance_m;
    const double final_heading_rad = poses.back().theta.radians();
    for (size_t i = poses.size() - 1; i > 0 && remaining_distance_m > 1e-6; --i) {
        const Pose2d& from = poses[i - 1];
        const Pose2d& to = poses[i];
        const double segment_length_m = std::hypot(to.x - from.x, to.y - from.y);
        if (segment_length_m <= 1e-6) {
            continue;
        }

        const double from_heading_delta =
            std::abs(normalizeAngleSigned(from.theta.radians() - final_heading_rad));
        const double to_heading_delta =
            std::abs(normalizeAngleSigned(to.theta.radians() - final_heading_rad));
        if (from_heading_delta > heading_tolerance_rad ||
            to_heading_delta > heading_tolerance_rad) {
            return false;
        }
        remaining_distance_m -= segment_length_m;
    }

    return true;
}

bool tailUsesOnlyMotionDirection(
    const std::vector<Pose2d>& poses,
    const std::vector<coastmotionplanning::common::MotionDirection>& segment_directions,
    double tail_distance_m,
    coastmotionplanning::common::MotionDirection expected_direction) {
    if (poses.size() < 2 || segment_directions.size() != poses.size() - 1) {
        return false;
    }

    double remaining_distance_m = tail_distance_m;
    for (size_t i = segment_directions.size(); i > 0 && remaining_distance_m > 1e-6; --i) {
        const size_t segment_idx = i - 1;
        const double segment_length_m = std::hypot(
            poses[segment_idx + 1].x - poses[segment_idx].x,
            poses[segment_idx + 1].y - poses[segment_idx].y);
        if (segment_length_m <= 1e-6) {
            continue;
        }

        if (segment_directions[segment_idx] != expected_direction) {
            return false;
        }
        remaining_distance_m -= segment_length_m;
    }

    return true;
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
        makeRectangle(min_x, min_y, max_x, max_y), "track");
    zone->addLaneFromPoints({
        {min_x + 1.0, min_y + ((max_y - min_y) * 0.25)},
        {max_x - 1.0, min_y + ((max_y - min_y) * 0.25)}
    });
    zone->addLaneFromPoints({
        {max_x - 1.0, min_y + ((max_y - min_y) * 0.75)},
        {min_x + 1.0, min_y + ((max_y - min_y) * 0.75)}
    });
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

TEST(HybridAStarPlannerTest, CrossZonePlanningShowsBehaviorSwitch) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 10.0, 4.0),
        makeManeuveringZone(10.0, -4.0, 20.0, 4.0, "parking_profile")
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(2.0, 0.0);
    request.goal = makePose(15.0, 0.0);
    request.initial_behavior_name = "primary_profile";

    const auto result = planner.plan(request);

    ASSERT_TRUE(result.success) << result.detail;
    ASSERT_FALSE(result.behavior_sequence.empty());
    EXPECT_EQ(result.behavior_sequence.front(), "primary_profile");
    EXPECT_EQ(result.behavior_sequence.back(), "parking_profile");
    EXPECT_NE(std::find(result.behavior_sequence.begin(),
                        result.behavior_sequence.end(),
                        "parking_profile"),
              result.behavior_sequence.end());
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

TEST(HybridAStarPlannerTest, AnalyticExpansionOutputHonorsMinimumSameMotionLength) {
    auto behavior_set =
        loadCustomPrimaryProfileBehaviorSet(false, 5.0, "Pro_XD", false, 0.0);
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

TEST(HybridAStarPlannerTest, GoalApproachStraightPreferenceKeepsFinalTailStraight) {
    auto behavior_set =
        loadCustomPrimaryProfileBehaviorSet(false, 1.0, "Pro_XD", false, 2.0);
    behavior_set.setMinimumPlanningTimeMs(5000);
    const auto& primary_profile = behavior_set.get("primary_profile");
    const double heading_tolerance_rad =
        Angle::from_degrees(primary_profile.planner.yaw_grid_resolution_deg).radians();
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
    EXPECT_TRUE(tailMaintainsHeading(
        result.poses,
        primary_profile.planner.goal_approach_straight_distance_m,
        heading_tolerance_rad));
}

TEST(HybridAStarPlannerTest, GoalApproachStraightPreferenceDisabledAllowsCurvedTail) {
    auto behavior_set =
        loadCustomPrimaryProfileBehaviorSet(false, 1.0, "Pro_XD", false, 0.0);
    behavior_set.setMinimumPlanningTimeMs(5000);
    const auto& primary_profile = behavior_set.get("primary_profile");
    const double heading_tolerance_rad =
        Angle::from_degrees(primary_profile.planner.yaw_grid_resolution_deg).radians();
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
    EXPECT_FALSE(tailMaintainsHeading(result.poses, 2.0, heading_tolerance_rad));
}

TEST(HybridAStarPlannerTest, ReverseStraightGoalApproachCountsAsStraight) {
    const auto behavior_set = loadCustomPrimaryProfileBehaviorSet(
        false,
        1.0,
        "Pro_XD",
        false,
        2.0,
        false,
        false);
    const auto& primary_profile = behavior_set.get("primary_profile");
    const double heading_tolerance_rad =
        Angle::from_degrees(primary_profile.planner.yaw_grid_resolution_deg).radians();
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeManeuveringZone(-2.0, -4.0, 8.0, 4.0, "primary_profile")
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(4.5, 0.0, 0.0);
    request.goal = makePose(2.0, 0.0, 0.0);
    request.initial_behavior_name = "primary_profile";

    const auto result = planner.plan(request);

    ASSERT_TRUE(result.success) << result.detail;
    EXPECT_TRUE(tailMaintainsHeading(
        result.poses,
        primary_profile.planner.goal_approach_straight_distance_m,
        heading_tolerance_rad));
    EXPECT_TRUE(tailUsesOnlyMotionDirection(
        result.poses,
        result.segment_directions,
        primary_profile.planner.goal_approach_straight_distance_m,
        coastmotionplanning::common::MotionDirection::Reverse));
}

TEST(HybridAStarPlannerTest, AnalyticExpansionRejectsTurningGoalApproachSuffix) {
    auto behavior_set =
        loadCustomPrimaryProfileBehaviorSet(false, 1.0, "Pro_XD", true, 2.0);
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

    ASSERT_NE(result.debug_trace, nullptr);
    EXPECT_TRUE(std::any_of(
        result.debug_trace->expansions.begin(),
        result.debug_trace->expansions.end(),
        [](const auto& expansion) {
            return expansion.analytic_attempted &&
                expansion.analytic_event.detail == "goal_approach_turning_suffix";
        }));
    if (result.success) {
        EXPECT_EQ(result.detail.find("analytic expansion"), std::string::npos);
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

TEST(HybridAStarPlannerTest, SameZoneDebugTraceUsesFinalGoalHeuristicOnly) {
    const PlannerBehaviorSet behavior_set =
        loadCustomPrimaryProfileBehaviorSet(false, 1.0, "Pro_XD", true);
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
    ASSERT_NE(result.debug_trace, nullptr);
    EXPECT_TRUE(result.debug_trace->stage_heuristic_layers.empty());
    ASSERT_FALSE(result.debug_trace->expansions.empty());
    EXPECT_TRUE(std::all_of(
        result.debug_trace->expansions.begin(),
        result.debug_trace->expansions.end(),
        [](const auto& expansion) {
            return expansion.heuristic_mode == "final_goal";
        }));
}

TEST(HybridAStarPlannerTest, CrossZoneDebugTraceIncludesStageHeuristicLayers) {
    const PlannerBehaviorSet behavior_set =
        loadCustomPrimaryProfileBehaviorSet(false, 1.0, "Pro_XD", true);
    const Car car = makeCar();
    std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        makeTrackZone(0.0, -4.0, 10.0, 4.0),
        makeManeuveringZone(30.0, -4.0, 40.0, 4.0, "parking_profile")
    };

    HybridAStarPlanner planner(car, zones, behavior_set);
    HybridAStarPlannerRequest request;
    request.start = makePose(2.0, 0.0, 0.0);
    request.goal = makePose(35.0, 0.0, 0.0);
    request.initial_behavior_name = "primary_profile";

    const auto result = planner.plan(request);

    ASSERT_NE(result.debug_trace, nullptr);
    ASSERT_EQ(result.debug_trace->stage_heuristic_layers.size(), 2u);
    EXPECT_TRUE(std::any_of(
        result.debug_trace->stage_heuristic_layers.begin(),
        result.debug_trace->stage_heuristic_layers.end(),
        [](const auto& stage_layer) {
            return stage_layer.source_frontier_id == 0 &&
                stage_layer.target_frontier_id == 1 &&
                stage_layer.seed_cell_count > 0;
        }));
    EXPECT_TRUE(std::any_of(
        result.debug_trace->stage_heuristic_layers.begin(),
        result.debug_trace->stage_heuristic_layers.end(),
        [](const auto& stage_layer) {
            return stage_layer.source_frontier_id == 1 &&
                stage_layer.target_frontier_id == 2 &&
                stage_layer.seed_cell_count > 0;
        }));
    ASSERT_FALSE(result.debug_trace->expansions.empty());
    EXPECT_TRUE(std::any_of(
        result.debug_trace->expansions.begin(),
        result.debug_trace->expansions.end(),
        [](const auto& expansion) {
            return expansion.heuristic_mode == "stage_frontier";
        }));
}

} // namespace
