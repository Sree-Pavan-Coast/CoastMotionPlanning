#include <gtest/gtest.h>

#include "coastmotionplanning/costs/zone_selector.hpp"
#include "coastmotionplanning/math/angle.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

using namespace coastmotionplanning;

class ZoneSelectorTest : public ::testing::Test {
protected:
    std::vector<std::shared_ptr<zones::Zone>> all_zones;

    void SetUp() override {
        // Zone A: maneuvering zone at [-5, 15] x [-5, 20]
        geometry::Polygon2d poly_a;
        poly_a.outer() = {
            geometry::Point2d(-5, -5), geometry::Point2d(15, -5),
            geometry::Point2d(15, 20), geometry::Point2d(-5, 20),
            geometry::Point2d(-5, -5)
        };
        all_zones.push_back(std::make_shared<zones::ManeuveringZone>(poly_a, "zone_a"));

        // Zone B: track road at [15, 100] x [0, 5]
        geometry::Polygon2d poly_b;
        poly_b.outer() = {
            geometry::Point2d(15, 0), geometry::Point2d(100, 0),
            geometry::Point2d(100, 5), geometry::Point2d(15, 5),
            geometry::Point2d(15, 0)
        };
        auto track = std::make_shared<zones::TrackMainRoad>(poly_b, "zone_b");
        all_zones.push_back(track);
    }
};

TEST_F(ZoneSelectorTest, StartAndGoalInSameZone) {
    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    math::Pose2d goal(5.0, 5.0, math::Angle::from_radians(0.0));

    costs::ZoneSelector selector;
    auto result = selector.select(start, goal, all_zones);

    EXPECT_EQ(result.selected_zones.size(), 1u);
    EXPECT_EQ(result.frontiers.size(), 1u);
    EXPECT_FALSE(result.search_boundary.outer().empty());
}

TEST_F(ZoneSelectorTest, StartAndGoalInDifferentZones) {
    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));     // In zone_a
    math::Pose2d goal(50.0, 2.5, math::Angle::from_radians(0.0));       // In zone_b

    costs::ZoneSelector selector;
    auto result = selector.select(start, goal, all_zones);

    EXPECT_EQ(result.selected_zones.size(), 2u);
    ASSERT_EQ(result.frontiers.size(), 3u);
    EXPECT_EQ(result.frontiers[0].frontier_id, 0u);
    EXPECT_EQ(result.frontiers[1].frontier_id, 1u);
    EXPECT_EQ(result.frontiers[2].frontier_id, 2u);
    EXPECT_FALSE(result.search_boundary.outer().empty());
}

TEST_F(ZoneSelectorTest, StartOutsideAllZones_Throws) {
    math::Pose2d start(200.0, 200.0, math::Angle::from_radians(0.0));  // Outside everything
    math::Pose2d goal(0.0, 0.0, math::Angle::from_radians(0.0));

    costs::ZoneSelector selector;
    EXPECT_THROW(selector.select(start, goal, all_zones), std::runtime_error);
}

TEST_F(ZoneSelectorTest, GoalOutsideAllZones_Throws) {
    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    math::Pose2d goal(200.0, 200.0, math::Angle::from_radians(0.0));

    costs::ZoneSelector selector;
    EXPECT_THROW(selector.select(start, goal, all_zones), std::runtime_error);
}

TEST_F(ZoneSelectorTest, ConcaveHullContainsBothZones) {
    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    math::Pose2d goal(50.0, 2.5, math::Angle::from_radians(0.0));

    costs::ZoneSelector selector;
    auto result = selector.select(start, goal, all_zones);

    // Verify start and goal are inside the boundary
    geometry::Point2d start_pt(start.x, start.y);
    geometry::Point2d goal_pt(goal.x, goal.y);

    EXPECT_TRUE(costs::ZoneSelector::isInsidePolygon(start_pt, result.search_boundary));
    EXPECT_TRUE(costs::ZoneSelector::isInsidePolygon(goal_pt, result.search_boundary));
}

TEST_F(ZoneSelectorTest, GappedZones_ConcaveHullBridgesGap) {
    // Add a zone with a gap from the others
    geometry::Polygon2d poly_c;
    poly_c.outer() = {
        geometry::Point2d(120, 0), geometry::Point2d(140, 0),
        geometry::Point2d(140, 10), geometry::Point2d(120, 10),
        geometry::Point2d(120, 0)
    };
    all_zones.push_back(std::make_shared<zones::ManeuveringZone>(poly_c, "zone_c"));

    math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));     // In zone_a
    math::Pose2d goal(130.0, 5.0, math::Angle::from_radians(0.0));     // In zone_c

    costs::ZoneSelector selector;
    auto result = selector.select(start, goal, all_zones);

    EXPECT_EQ(result.selected_zones.size(), 2u);
    EXPECT_EQ(result.frontiers.size(), 3u);
    EXPECT_TRUE(costs::ZoneSelector::isInsidePolygon(
        geometry::Point2d(0.0, 0.0), result.search_boundary));
    EXPECT_TRUE(costs::ZoneSelector::isInsidePolygon(
        geometry::Point2d(130.0, 5.0), result.search_boundary));
    EXPECT_TRUE(costs::ZoneSelector::isInsidePolygon(
        geometry::Point2d(70.0, 5.0), result.search_boundary));
}

TEST_F(ZoneSelectorTest, FindContainingZone_ReturnsCorrectZone) {
    geometry::Point2d pt_in_a(0.0, 0.0);
    auto zone = costs::ZoneSelector::findContainingZone(pt_in_a, all_zones);
    ASSERT_NE(zone, nullptr);
    EXPECT_EQ(zone->getName().value_or(""), "zone_a");

    geometry::Point2d pt_in_b(50.0, 2.5);
    zone = costs::ZoneSelector::findContainingZone(pt_in_b, all_zones);
    ASSERT_NE(zone, nullptr);
    EXPECT_EQ(zone->getName().value_or(""), "zone_b");

    geometry::Point2d pt_outside(200.0, 200.0);
    zone = costs::ZoneSelector::findContainingZone(pt_outside, all_zones);
    EXPECT_EQ(zone, nullptr);
}
