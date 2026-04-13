#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "coastmotionplanning/math/angle.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/planning/planner_behavior_resolver.hpp"
#include "coastmotionplanning/planning/planner_behavior_set.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

namespace {

using coastmotionplanning::geometry::Polygon2d;
using coastmotionplanning::math::Angle;
using coastmotionplanning::math::Pose2d;
using coastmotionplanning::planning::PlannerBehaviorResolver;
using coastmotionplanning::planning::PlannerBehaviorSet;
using coastmotionplanning::planning::ResolvedPlannerBehavior;

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

Pose2d makePose(double x, double y) {
    return Pose2d{x, y, Angle::from_degrees(0.0)};
}

TEST(PlannerBehaviorCatalogTest, LoadsTypedProfileValuesFromConfig) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();

    const auto& primary = behavior_set.get("primary_profile");
    const auto& parking = behavior_set.get("parking_profile");

    EXPECT_EQ(primary.planner.max_planning_time_ms, 200);
    EXPECT_DOUBLE_EQ(primary.planner.step_size_m, 0.5);
    EXPECT_DOUBLE_EQ(primary.costmap.resolution_m, 0.1);
    EXPECT_EQ(primary.motion_primitives.num_angle_bins, 72);
    EXPECT_EQ(primary.collision_checker.collision_mode, "strict");
    EXPECT_TRUE(primary.isLayerActive("combined_cost"));

    EXPECT_EQ(parking.planner.max_planning_time_ms, 100);
    EXPECT_DOUBLE_EQ(parking.planner.xy_grid_resolution_m, 0.05);
    EXPECT_DOUBLE_EQ(parking.planner.yaw_grid_resolution_deg, 2.5);
    EXPECT_EQ(parking.motion_primitives.num_angle_bins, 144);
    EXPECT_TRUE(parking.isLayerActive("inflation"));
    EXPECT_FALSE(parking.isLayerActive("lane_centerline_cost"));
}

TEST(PlannerBehaviorCatalogTest, MissingProfileLookupThrows) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();

    EXPECT_THROW(behavior_set.get("missing_profile"), std::runtime_error);
}

TEST(PlannerBehaviorResolverTest, KeepsCurrentBehaviorWhenSuccessorRemainsInCurrentZone) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const auto current_zone = makeManeuveringZone(0.0, 0.0, 10.0, 10.0);
    const auto other_zone = makeTrackZone(10.0, 0.0, 20.0, 10.0);
    const std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        current_zone, other_zone};

    const ResolvedPlannerBehavior resolved = PlannerBehaviorResolver::resolve(
        makePose(5.0, 5.0), current_zone, "parking_profile", zones, behavior_set);

    EXPECT_FALSE(resolved.switched_zone);
    EXPECT_EQ(resolved.zone, current_zone);
    EXPECT_EQ(resolved.behavior_name, "parking_profile");
    ASSERT_NE(resolved.profile, nullptr);
    EXPECT_EQ(resolved.profile->planner.max_planning_time_ms, 100);
}

TEST(PlannerBehaviorResolverTest, SwitchesWhenSuccessorMovesIntoAnotherZoneInterior) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const auto current_zone = makeManeuveringZone(0.0, 0.0, 10.0, 10.0);
    const auto other_zone = makeTrackZone(10.0, 0.0, 20.0, 10.0);
    const std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        current_zone, other_zone};

    const ResolvedPlannerBehavior resolved = PlannerBehaviorResolver::resolve(
        makePose(15.0, 5.0), current_zone, "parking_profile", zones, behavior_set);

    EXPECT_TRUE(resolved.switched_zone);
    EXPECT_EQ(resolved.zone, other_zone);
    EXPECT_EQ(resolved.behavior_name, "primary_profile");
    ASSERT_NE(resolved.profile, nullptr);
    EXPECT_EQ(resolved.profile->motion_primitives.num_angle_bins, 72);
}

TEST(PlannerBehaviorResolverTest, KeepsCurrentBehaviorOnZoneBoundary) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const auto current_zone = makeManeuveringZone(0.0, 0.0, 10.0, 10.0);
    const auto other_zone = makeTrackZone(10.0, 0.0, 20.0, 10.0);
    const std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        current_zone, other_zone};

    const ResolvedPlannerBehavior resolved = PlannerBehaviorResolver::resolve(
        makePose(10.0, 5.0), current_zone, "parking_profile", zones, behavior_set);

    EXPECT_FALSE(resolved.switched_zone);
    EXPECT_EQ(resolved.zone, current_zone);
    EXPECT_EQ(resolved.behavior_name, "parking_profile");
}

TEST(PlannerBehaviorResolverTest, ThrowsWhenResolvedZoneBehaviorIsMissingFromCatalog) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const auto current_zone = makeManeuveringZone(0.0, 0.0, 10.0, 10.0);
    const auto other_zone = makeTrackZone(10.0, 0.0, 20.0, 10.0, "missing_profile");
    const std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        current_zone, other_zone};

    EXPECT_THROW(
        PlannerBehaviorResolver::resolve(
            makePose(15.0, 5.0), current_zone, "parking_profile", zones, behavior_set),
        std::runtime_error);
}

TEST(PlannerBehaviorResolverTest, OverlappingZonesPreserveFirstMatchOrdering) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const auto first_zone = makeManeuveringZone(0.0, 0.0, 10.0, 10.0);
    const auto current_zone = makeTrackZone(2.0, 2.0, 8.0, 8.0);
    const std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        first_zone, current_zone};

    const ResolvedPlannerBehavior resolved = PlannerBehaviorResolver::resolve(
        makePose(5.0, 5.0), current_zone, "primary_profile", zones, behavior_set);

    EXPECT_TRUE(resolved.switched_zone);
    EXPECT_EQ(resolved.zone, first_zone);
    EXPECT_EQ(resolved.behavior_name, "parking_profile");
    ASSERT_NE(resolved.profile, nullptr);
    EXPECT_EQ(resolved.profile->planner.max_planning_time_ms, 100);
}

} // namespace
