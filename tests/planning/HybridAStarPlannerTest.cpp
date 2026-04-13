#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "coastmotionplanning/math/angle.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/motion_primitives/motion_primitive_types.hpp"
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

PlannerBehaviorSet loadBehaviorSet() {
    return PlannerBehaviorSet::loadFromFile(repoPath("configs/planner_behaviors.yaml"));
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

TEST(HybridAStarPlannerPolicyTest, ForwardOnlyZonesUseDubinsAndPruneReverse) {
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
        coastmotionplanning::costs::HeuristicModel::DUBINS);
    EXPECT_TRUE(HybridAStarPlanner::isPrimitiveAllowed(
        coastmotionplanning::motion_primitives::TurnDirection::FORWARD, resolved));
    EXPECT_FALSE(HybridAStarPlanner::isPrimitiveAllowed(
        coastmotionplanning::motion_primitives::TurnDirection::REVERSE, resolved));
    EXPECT_FALSE(HybridAStarPlanner::isPrimitiveAllowed(
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
        makeManeuveringZone(10.0, -4.0, 20.0, 4.0)
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

} // namespace
