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

PlannerBehaviorSet loadCustomPrimaryProfileBehaviorSet(bool only_forward_path,
                                                       double min_same_motion_length_m,
                                                       const std::string& robot_name = "E_Transit") {
    const std::filesystem::path temp_dir =
        std::filesystem::temp_directory_path() / "coastmotionplanning_hybrid_astar_test";
    std::filesystem::create_directories(temp_dir);

    std::ofstream master_file(temp_dir / "master_params.yaml");
    master_file << readFile(repoPath("configs/master_params.yaml"));
    master_file.close();

    std::string behaviors = readFile(repoPath("configs/planner_behaviors.yaml"));
    const std::string only_forward_false = "      only_forward_path: false";
    const size_t only_forward_pos = behaviors.find(only_forward_false);
    if (only_forward_pos == std::string::npos) {
        throw std::runtime_error("Failed to locate only_forward_path in planner_behaviors.yaml");
    }
    behaviors.replace(
        only_forward_pos,
        only_forward_false.size(),
        std::string("      only_forward_path: ") + (only_forward_path ? "true" : "false"));

    const std::string current_min_same_motion = "      min_path_len_in_same_motion: 1.0";
    const size_t min_same_motion_pos = behaviors.find(current_min_same_motion);
    if (min_same_motion_pos == std::string::npos) {
        throw std::runtime_error(
            "Failed to locate min_path_len_in_same_motion in planner_behaviors.yaml");
    }
    behaviors.replace(
        min_same_motion_pos,
        current_min_same_motion.size(),
        "      min_path_len_in_same_motion: " + std::to_string(min_same_motion_length_m));

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
        track_zone,
        "primary_profile",
        &behavior_set.get("primary_profile"),
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
        maneuvering_zone,
        "parking_profile",
        &behavior_set.get("parking_profile"),
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
        maneuvering_zone,
        "custom_forward_only",
        &forward_only_profile,
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
    const PlannerBehaviorSet behavior_set =
        loadCustomPrimaryProfileBehaviorSet(false, 5.0, "Pro_XD");
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

} // namespace
