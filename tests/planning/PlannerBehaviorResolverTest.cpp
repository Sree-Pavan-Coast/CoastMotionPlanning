#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/geometry/algorithms/envelope.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/costs/zone_constraints_layer.hpp"
#include "coastmotionplanning/math/angle.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/planning/planner_behavior_resolver.hpp"
#include "coastmotionplanning/planning/planner_behavior_set.hpp"
#include "coastmotionplanning/costs/zone_selector.hpp"
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

Pose2d makePose(double x, double y) {
    return Pose2d{x, y, Angle::from_degrees(0.0)};
}

grid_map::GridMap buildZoneConstraintCostmap(
    const coastmotionplanning::costs::ZoneSelectionResult& selection) {
    coastmotionplanning::geometry::Box2d bbox;
    coastmotionplanning::geometry::bg::envelope(selection.search_boundary, bbox);

    constexpr double kMarginM = 1.0;
    const double min_x =
        coastmotionplanning::geometry::bg::get<coastmotionplanning::geometry::bg::min_corner, 0>(
            bbox) - kMarginM;
    const double min_y =
        coastmotionplanning::geometry::bg::get<coastmotionplanning::geometry::bg::min_corner, 1>(
            bbox) - kMarginM;
    const double max_x =
        coastmotionplanning::geometry::bg::get<coastmotionplanning::geometry::bg::max_corner, 0>(
            bbox) + kMarginM;
    const double max_y =
        coastmotionplanning::geometry::bg::get<coastmotionplanning::geometry::bg::max_corner, 1>(
            bbox) + kMarginM;

    grid_map::GridMap costmap;
    costmap.setFrameId("world");
    costmap.setGeometry(
        grid_map::Length(max_x - min_x, max_y - min_y),
        0.1,
        grid_map::Position((min_x + max_x) / 2.0, (min_y + max_y) / 2.0));
    coastmotionplanning::costs::ZoneConstraintsLayer::build(costmap, selection);
    return costmap;
}

coastmotionplanning::costs::ZoneSelectionResult buildSelection(
    const Pose2d& start,
    const Pose2d& goal,
    const std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>>& zones,
    const std::string& initial_behavior = "parking_profile",
    const std::string& transition_behavior = "maneuver_to_track_profile") {
    coastmotionplanning::costs::ZoneSelector selector;
    return selector.select(
        start,
        goal,
        zones,
        0.0,
        initial_behavior,
        transition_behavior);
}

TEST(PlannerBehaviorCatalogTest, LoadsTypedProfileValuesFromConfig) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();

    const auto& primary = behavior_set.get("primary_profile");
    const auto& parking = behavior_set.get("parking_profile");

    EXPECT_EQ(primary.planner.max_planning_time_ms, 2000);
    EXPECT_DOUBLE_EQ(primary.planner.step_size_m, 0.5);
    EXPECT_TRUE(primary.planner.only_forward_path);
    EXPECT_DOUBLE_EQ(primary.planner.weight_gear_change, 4.0);
    EXPECT_DOUBLE_EQ(primary.planner.min_path_len_in_same_motion, 5.0);
    EXPECT_TRUE(primary.planner.analytic_shot);
    EXPECT_DOUBLE_EQ(primary.planner.weight_lane_centerline, 1.0);
    EXPECT_DOUBLE_EQ(primary.costmap.resolution_m, 0.1);
    EXPECT_EQ(primary.motion_primitives.num_angle_bins, 72);
    EXPECT_EQ(primary.collision_checker.collision_mode, "strict");
    EXPECT_TRUE(primary.isLayerActive("combined_cost"));
    EXPECT_TRUE(behavior_set.contains("maneuver_to_track_profile"));

    EXPECT_EQ(parking.planner.max_planning_time_ms, 100);
    EXPECT_DOUBLE_EQ(parking.planner.xy_grid_resolution_m, 0.05);
    EXPECT_DOUBLE_EQ(parking.planner.yaw_grid_resolution_deg, 2.5);
    EXPECT_FALSE(parking.planner.only_forward_path);
    EXPECT_DOUBLE_EQ(parking.planner.weight_gear_change, 1.0);
    EXPECT_DOUBLE_EQ(parking.planner.min_path_len_in_same_motion, 0.5);
    EXPECT_EQ(parking.motion_primitives.num_angle_bins, 144);
    EXPECT_TRUE(parking.isLayerActive("inflation"));
    EXPECT_FALSE(parking.isLayerActive("lane_centerline_cost"));

    const auto& track = behavior_set.get("track_main_road_profile");
    EXPECT_FALSE(track.planner.analytic_shot);
    EXPECT_DOUBLE_EQ(track.planner.weight_lane_centerline, 3.0);
    EXPECT_DOUBLE_EQ(track.planner.lane_heading_bias_weight, 2.0);
    EXPECT_TRUE(track.planner.lane_primitive_suppression);
    EXPECT_TRUE(track.isLayerActive("lane_centerline_cost"));
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
    const auto selection = buildSelection(makePose(5.0, 5.0), makePose(15.0, 5.0), zones);
    const grid_map::GridMap costmap = buildZoneConstraintCostmap(selection);

    const ResolvedPlannerBehavior resolved = PlannerBehaviorResolver::resolve(
        makePose(5.0, 5.0),
        costmap,
        0,
        current_zone,
        "parking_profile",
        selection.frontiers,
        behavior_set);

    EXPECT_FALSE(resolved.switched_zone);
    EXPECT_FALSE(resolved.switched_frontier);
    EXPECT_EQ(resolved.frontier_id, 0u);
    EXPECT_EQ(resolved.zone, current_zone);
    EXPECT_EQ(resolved.behavior_name, "parking_profile");
    ASSERT_NE(resolved.profile, nullptr);
    EXPECT_EQ(resolved.profile->planner.max_planning_time_ms, 100);
}

TEST(PlannerBehaviorResolverTest, SwitchesZoneButKeepsCurrentBehaviorWhenSuccessorZoneHasNoAssignment) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const auto current_zone = makeManeuveringZone(0.0, 0.0, 10.0, 10.0);
    const auto other_zone = makeTrackZone(10.0, 0.0, 20.0, 10.0);
    const std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        current_zone, other_zone};
    const auto selection = buildSelection(makePose(5.0, 5.0), makePose(15.0, 5.0), zones);
    const grid_map::GridMap costmap = buildZoneConstraintCostmap(selection);

    const ResolvedPlannerBehavior resolved = PlannerBehaviorResolver::resolve(
        makePose(15.0, 5.0),
        costmap,
        0,
        current_zone,
        "parking_profile",
        selection.frontiers,
        behavior_set);

    EXPECT_TRUE(resolved.switched_zone);
    EXPECT_TRUE(resolved.switched_frontier);
    EXPECT_EQ(resolved.frontier_id, 2u);
    EXPECT_EQ(resolved.zone, other_zone);
    EXPECT_EQ(resolved.behavior_name, "parking_profile");
    ASSERT_NE(resolved.profile, nullptr);
    EXPECT_EQ(resolved.profile->motion_primitives.num_angle_bins, 144);
}

TEST(PlannerBehaviorResolverTest, SwitchesToTransitionFrontierBehaviorInGapSpace) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const auto current_zone = makeManeuveringZone(0.0, 0.0, 10.0, 10.0);
    const auto other_zone = makeTrackZone(12.0, 0.0, 22.0, 10.0);
    const std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        current_zone, other_zone};
    const auto selection = buildSelection(makePose(5.0, 5.0), makePose(15.0, 5.0), zones);
    const grid_map::GridMap costmap = buildZoneConstraintCostmap(selection);

    const ResolvedPlannerBehavior resolved = PlannerBehaviorResolver::resolve(
        makePose(11.0, 5.0),
        costmap,
        0,
        current_zone,
        "parking_profile",
        selection.frontiers,
        behavior_set);

    EXPECT_FALSE(resolved.switched_zone);
    EXPECT_TRUE(resolved.switched_frontier);
    EXPECT_EQ(resolved.frontier_id, 1u);
    EXPECT_EQ(resolved.zone, nullptr);
    EXPECT_EQ(resolved.behavior_name, "maneuver_to_track_profile");
}

TEST(PlannerBehaviorResolverTest, ThrowsWhenResolvedZoneBehaviorIsMissingFromCatalog) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const auto current_zone = makeManeuveringZone(0.0, 0.0, 10.0, 10.0);
    const auto other_zone = makeTrackZone(10.0, 0.0, 20.0, 10.0, "missing_profile");
    const std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        current_zone, other_zone};
    const auto selection = buildSelection(makePose(5.0, 5.0), makePose(15.0, 5.0), zones);
    const grid_map::GridMap costmap = buildZoneConstraintCostmap(selection);

    EXPECT_THROW(
        PlannerBehaviorResolver::resolve(
            makePose(15.0, 5.0),
            costmap,
            0,
            current_zone,
            "parking_profile",
            selection.frontiers,
            behavior_set),
        std::runtime_error);
}

TEST(PlannerBehaviorResolverTest, OverlappingZonesPreserveFirstMatchOrdering) {
    const PlannerBehaviorSet behavior_set = loadBehaviorSet();
    const auto first_zone = makeManeuveringZone(0.0, 0.0, 10.0, 10.0);
    const auto current_zone = makeTrackZone(2.0, 2.0, 8.0, 8.0);
    const std::vector<std::shared_ptr<coastmotionplanning::zones::Zone>> zones{
        first_zone, current_zone};
    const auto selection = buildSelection(
        makePose(3.0, 3.0),
        makePose(7.0, 7.0),
        zones,
        "primary_profile");
    const grid_map::GridMap costmap = buildZoneConstraintCostmap(selection);

    const ResolvedPlannerBehavior resolved = PlannerBehaviorResolver::resolve(
        makePose(5.0, 5.0),
        costmap,
        0,
        current_zone,
        "primary_profile",
        selection.frontiers,
        behavior_set);

    EXPECT_TRUE(resolved.switched_zone);
    EXPECT_FALSE(resolved.switched_frontier);
    EXPECT_EQ(resolved.zone, first_zone);
    EXPECT_EQ(resolved.behavior_name, "primary_profile");
    ASSERT_NE(resolved.profile, nullptr);
    EXPECT_EQ(resolved.profile->planner.max_planning_time_ms, 2000);
}

} // namespace
