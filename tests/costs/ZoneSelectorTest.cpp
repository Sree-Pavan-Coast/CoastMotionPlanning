#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <boost/geometry/algorithms/area.hpp>

#include "coastmotionplanning/costs/zone_selector.hpp"
#include "coastmotionplanning/math/angle.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

using namespace coastmotionplanning;

namespace {

geometry::Polygon2d makeRectangle(double min_x, double min_y, double max_x, double max_y) {
    geometry::Polygon2d polygon;
    polygon.outer() = {
        geometry::Point2d(min_x, min_y), geometry::Point2d(max_x, min_y),
        geometry::Point2d(max_x, max_y), geometry::Point2d(min_x, max_y),
        geometry::Point2d(min_x, min_y)
    };
    return polygon;
}

double polygonAreaMagnitude(const geometry::Polygon2d& polygon) {
    return std::abs(geometry::bg::area(polygon));
}

} // namespace

class ZoneSelectorTest : public ::testing::Test {
protected:
    std::vector<std::shared_ptr<zones::Zone>> all_zones;

    void SetUp() override {
        all_zones.push_back(std::make_shared<zones::ManeuveringZone>(
            makeRectangle(-5.0, -5.0, 15.0, 20.0), "zone_a"));

        all_zones.push_back(std::make_shared<zones::TrackMainRoad>(
            makeRectangle(15.0, 0.0, 100.0, 5.0), "zone_b"));
    }
};

TEST_F(ZoneSelectorTest, StartAndGoalInSameZoneUseThatZoneExactly) {
    const math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    const math::Pose2d goal(5.0, 5.0, math::Angle::from_radians(0.0));

    costs::ZoneSelector selector;
    const auto result = selector.select(start, goal, all_zones);

    ASSERT_EQ(result.selected_zones.size(), 1u);
    ASSERT_EQ(result.frontiers.size(), 1u);
    EXPECT_EQ(result.frontiers[0].frontier_id, 0u);
    EXPECT_EQ(result.frontiers[0].role, costs::SearchFrontierRole::StartZone);
    EXPECT_NEAR(
        polygonAreaMagnitude(result.search_boundary),
        polygonAreaMagnitude(all_zones[0]->getPolygon()),
        1e-6);
}

TEST_F(ZoneSelectorTest, StartAndGoalInDifferentDirectlyConnectedZonesUseExactUnion) {
    const math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    const math::Pose2d goal(50.0, 2.5, math::Angle::from_radians(0.0));

    costs::ZoneSelector selector;
    const auto result = selector.select(start, goal, all_zones);

    ASSERT_EQ(result.selected_zones.size(), 2u);
    ASSERT_EQ(result.frontiers.size(), 2u);
    EXPECT_EQ(result.frontiers[0].frontier_id, 0u);
    EXPECT_EQ(result.frontiers[1].frontier_id, 1u);
    EXPECT_EQ(result.frontiers[0].role, costs::SearchFrontierRole::StartZone);
    EXPECT_EQ(result.frontiers[1].role, costs::SearchFrontierRole::GoalZone);
    EXPECT_TRUE(costs::ZoneSelector::isInsidePolygon(
        geometry::Point2d(start.x, start.y), result.search_boundary));
    EXPECT_TRUE(costs::ZoneSelector::isInsidePolygon(
        geometry::Point2d(goal.x, goal.y), result.search_boundary));
    EXPECT_FALSE(costs::ZoneSelector::isInsidePolygon(
        geometry::Point2d(20.0, 10.0), result.search_boundary));
    EXPECT_FALSE(costs::ZoneSelector::isInsidePolygon(
        geometry::Point2d(50.0, 10.0), result.search_boundary));
    EXPECT_NEAR(polygonAreaMagnitude(result.search_boundary), 925.0, 1e-6);
}

TEST_F(ZoneSelectorTest, StartOutsideAllZonesThrows) {
    const math::Pose2d start(200.0, 200.0, math::Angle::from_radians(0.0));
    const math::Pose2d goal(0.0, 0.0, math::Angle::from_radians(0.0));

    costs::ZoneSelector selector;
    EXPECT_THROW(selector.select(start, goal, all_zones), std::runtime_error);
}

TEST_F(ZoneSelectorTest, GoalOutsideAllZonesThrows) {
    const math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    const math::Pose2d goal(200.0, 200.0, math::Angle::from_radians(0.0));

    costs::ZoneSelector selector;
    EXPECT_THROW(selector.select(start, goal, all_zones), std::runtime_error);
}

TEST_F(ZoneSelectorTest, DisconnectedZonesThrowInsteadOfBridgingTheGap) {
    all_zones.push_back(std::make_shared<zones::ManeuveringZone>(
        makeRectangle(120.0, 0.0, 140.0, 10.0), "zone_c"));

    const math::Pose2d start(0.0, 0.0, math::Angle::from_radians(0.0));
    const math::Pose2d goal(130.0, 5.0, math::Angle::from_radians(0.0));

    costs::ZoneSelector selector;
    EXPECT_THROW(selector.select(start, goal, all_zones), std::runtime_error);
}

TEST(ZoneSelectorGeometryTest, PointTouchingZonesAreNotDirectlyConnected) {
    const std::vector<std::shared_ptr<zones::Zone>> zones{
        std::make_shared<zones::ManeuveringZone>(makeRectangle(0.0, 0.0, 10.0, 10.0), "zone_a"),
        std::make_shared<zones::ManeuveringZone>(makeRectangle(10.0, 10.0, 20.0, 20.0), "zone_b")
    };

    const math::Pose2d start(5.0, 5.0, math::Angle::from_radians(0.0));
    const math::Pose2d goal(15.0, 15.0, math::Angle::from_radians(0.0));

    costs::ZoneSelector selector;
    EXPECT_THROW(selector.select(start, goal, zones), std::runtime_error);
}

TEST(ZoneSelectorGeometryTest, OverlappingZonesAreDirectlyConnected) {
    const std::vector<std::shared_ptr<zones::Zone>> zones{
        std::make_shared<zones::ManeuveringZone>(makeRectangle(0.0, 0.0, 10.0, 10.0), "zone_a"),
        std::make_shared<zones::ManeuveringZone>(makeRectangle(8.0, 0.0, 18.0, 10.0), "zone_b")
    };

    const math::Pose2d start(5.0, 5.0, math::Angle::from_radians(0.0));
    const math::Pose2d goal(15.0, 5.0, math::Angle::from_radians(0.0));

    costs::ZoneSelector selector;
    const auto result = selector.select(start, goal, zones);

    ASSERT_EQ(result.selected_zones.size(), 2u);
    ASSERT_EQ(result.frontiers.size(), 2u);
    EXPECT_NEAR(polygonAreaMagnitude(result.search_boundary), 180.0, 1e-6);
}

TEST(ZoneSelectorGeometryTest, ConnectivityIndexTreatsEdgesAndOverlapAsConnectedOnly) {
    const std::vector<std::shared_ptr<zones::Zone>> zones{
        std::make_shared<zones::ManeuveringZone>(makeRectangle(0.0, 0.0, 10.0, 10.0), "zone_a"),
        std::make_shared<zones::ManeuveringZone>(makeRectangle(10.0, 0.0, 20.0, 10.0), "zone_b"),
        std::make_shared<zones::ManeuveringZone>(makeRectangle(8.0, 8.0, 18.0, 18.0), "zone_c"),
        std::make_shared<zones::ManeuveringZone>(makeRectangle(20.0, 10.0, 30.0, 20.0), "zone_d"),
        std::make_shared<zones::ManeuveringZone>(makeRectangle(40.0, 0.0, 50.0, 10.0), "zone_e")
    };

    const auto connectivity = costs::ZoneSelector::buildConnectivityIndex(zones);

    EXPECT_TRUE(connectivity.areDirectlyConnected(0u, 1u));
    EXPECT_TRUE(connectivity.areDirectlyConnected(0u, 2u));
    EXPECT_TRUE(connectivity.areDirectlyConnected(1u, 2u));
    EXPECT_FALSE(connectivity.areDirectlyConnected(1u, 3u));
    EXPECT_FALSE(connectivity.areDirectlyConnected(0u, 4u));
    EXPECT_TRUE(connectivity.isIsolated(3u));
    EXPECT_TRUE(connectivity.isIsolated(4u));
    EXPECT_FALSE(connectivity.isIsolated(0u));
}

TEST_F(ZoneSelectorTest, FindContainingZoneReturnsCorrectZone) {
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
