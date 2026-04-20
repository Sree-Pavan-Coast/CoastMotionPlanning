#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <optional>

#include "coastmotionplanning/map/map_parser.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

using namespace coastmotionplanning::map;

class MapParserTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::ofstream ofs("valid_lat_lon_map.yaml");
        ofs << "maps:\n"
            << "  name: \"Lat/Lon Map\"\n"
            << "  geographic_origin:\n"
            << "    latitude: 49.3175\n"
            << "    longitude: -122.3958\n"
            << "    altitude: 0.0\n"
            << "  model_metric: 1.0\n"
            << "zones:\n"
            << "  - name: \"lat_lon_zone\"\n"
            << "    type: \"ManeuveringZone\"\n"
            << "    planner_behavior: \"parking_profile\"\n"
            << "    coordinate_type: \"lat_long\"\n"
            << "    polygon:\n"
            << "      - [49.3175, -122.3958]\n"
            << "      - [49.3180, -122.3958]\n";
        ofs.close();

        std::ofstream ofs_world("valid_world_map.yaml");
        ofs_world << "maps:\n"
                  << "  name: \"World Map\"\n"
                  << "zones:\n"
                  << "  - name: \"world_zone\"\n"
                  << "    type: \"ManeuveringZone\"\n"
                  << "    planner_behavior: \"primary_profile\"\n"
                  << "    coordinate_type: \"world\"\n"
                  << "    polygon:\n"
                  << "      - [1.0, 2.0]\n"
                  << "      - [3.0, 4.0]\n"
                  << "  - name: \"multi_lane_zone\"\n"
                  << "    type: \"TrackMainRoad\"\n"
                  << "    planner_behavior: \"primary_profile\"\n"
                  << "    coordinate_type: \"world\"\n"
                  << "    polygon:\n"
                  << "      - [0.0, 0.0]\n"
                  << "      - [10.0, 0.0]\n"
                  << "      - [10.0, 6.0]\n"
                  << "      - [0.0, 6.0]\n"
                  << "    lane:\n"
                  << "      segments:\n"
                  << "        - id: 1\n"
                  << "          offset: 1.0\n"
                  << "          center_waypoints:\n"
                  << "            - [1.0, 3.0]\n"
                  << "            - [9.0, 3.0]\n";
        ofs_world.close();

        std::ofstream ofs_scaled("scaled_lat_lon_map.yaml");
        ofs_scaled << "maps:\n"
                   << "  name: \"Scaled Lat/Lon Map\"\n"
                   << "  geographic_origin:\n"
                   << "    latitude: 49.3175\n"
                   << "    longitude: -122.3958\n"
                   << "    altitude: 0.0\n"
                   << "  model_metric: 2.0\n"
                   << "zones:\n"
                   << "  - name: \"scaled_lat_lon_zone\"\n"
                   << "    type: \"ManeuveringZone\"\n"
                   << "    planner_behavior: \"relaxed_profile\"\n"
                   << "    coordinate_type: \"lat_long\"\n"
                   << "    polygon:\n"
                   << "      - [49.3175, -122.3958]\n"
                   << "      - [49.3180, -122.3958]\n";
        ofs_scaled.close();

        std::ofstream ofs_long_lat("valid_long_lat_map.yaml");
        ofs_long_lat << "maps:\n"
                     << "  name: \"Long/Lat Map\"\n"
                     << "  geographic_origin:\n"
                     << "    latitude: 49.3175\n"
                     << "    longitude: -122.3958\n"
                     << "    altitude: 0.0\n"
                     << "  model_metric: 1.0\n"
                     << "zones:\n"
                     << "  - name: \"long_lat_zone\"\n"
                     << "    type: \"ManeuveringZone\"\n"
                     << "    planner_behavior: \"primary_profile\"\n"
                     << "    coordinate_type: \"long_lat\"\n"
                     << "    polygon:\n"
                     << "      - [-122.3958, 49.3175]\n"
                     << "      - [-122.3958, 49.3180]\n";
        ofs_long_lat.close();

        std::ofstream ofs_inv("invalid_map.yaml");
        ofs_inv << "maps:\n"
                << "  name: \"Invalid Map\"\n"
                << "  geographic_origin:\n"
                << "    latitude: 49.3175\n"
                << "    longitude: -122.3958\n"
                << "zones:\n"
                << "  - name: \"invalid_zone\"\n"
                << "    type: \"ManeuveringZone\"\n"
                << "    planner_behavior: \"parking_profile\"\n"
                << "    coordinate_type: \"lat_long\"\n"
                << "    polygon:\n"
                << "      - [49.3175, -122.3958]\n";
        ofs_inv.close();

        std::ofstream ofs_default("default_behavior_map.yaml");
        ofs_default << "maps:\n"
                    << "  name: \"Default Behavior Map\"\n"
                    << "zones:\n"
                    << "  - name: \"default_track\"\n"
                    << "    type: \"TrackMainRoad\"\n"
                    << "    planner_behavior: \"default\"\n"
                    << "    coordinate_type: \"world\"\n"
                    << "    polygon:\n"
                    << "      - [0.0, 0.0]\n"
                    << "      - [10.0, 0.0]\n"
                    << "      - [10.0, 5.0]\n"
                    << "      - [0.0, 5.0]\n"
                    << "    lane:\n"
                    << "      segments:\n"
                    << "        - id: default_track_center\n"
                    << "          offset: 1.0\n"
                    << "          center_waypoints:\n"
                    << "            - [1.0, 2.5]\n"
                    << "            - [9.0, 2.5]\n"
                    << "  - name: \"implicit_default_maneuvering\"\n"
                    << "    type: \"ManeuveringZone\"\n"
                    << "    coordinate_type: \"world\"\n"
                    << "    polygon:\n"
                    << "      - [20.0, 0.0]\n"
                    << "      - [30.0, 0.0]\n"
                    << "      - [30.0, 5.0]\n"
                    << "      - [20.0, 5.0]\n";
        ofs_default.close();

        std::ofstream ofs_dup("duplicate_zone_map.yaml");
        ofs_dup << "maps:\n"
                << "  name: \"Duplicate Zone Map\"\n"
                << "zones:\n"
                << "  - name: \"dup_zone\"\n"
                << "    type: \"ManeuveringZone\"\n"
                << "    coordinate_type: \"world\"\n"
                << "    polygon:\n"
                << "      - [0.0, 0.0]\n"
                << "      - [1.0, 0.0]\n"
                << "  - name: \"dup_zone\"\n"
                << "    type: \"TrackMainRoad\"\n"
                << "    coordinate_type: \"world\"\n"
                << "    polygon:\n"
                << "      - [2.0, 0.0]\n"
                << "      - [3.0, 0.0]\n";
        ofs_dup.close();

        std::ofstream ofs_legacy("legacy_id_map.yaml");
        ofs_legacy << "maps:\n"
                   << "  name: \"Legacy ID Map\"\n"
                   << "zones:\n"
                   << "  - id: \"zone_a\"\n"
                   << "    name: \"legacy_zone\"\n"
                   << "    type: \"ManeuveringZone\"\n"
                   << "    coordinate_type: \"world\"\n"
                   << "    polygon:\n"
                   << "      - [0.0, 0.0]\n"
                   << "      - [1.0, 0.0]\n";
        ofs_legacy.close();
    }

    void TearDown() override {
        std::remove("valid_lat_lon_map.yaml");
        std::remove("valid_world_map.yaml");
        std::remove("scaled_lat_lon_map.yaml");
        std::remove("valid_long_lat_map.yaml");
        std::remove("invalid_map.yaml");
        std::remove("default_behavior_map.yaml");
        std::remove("duplicate_zone_map.yaml");
        std::remove("legacy_id_map.yaml");
    }
};

TEST_F(MapParserTest, ThrowsOnMissingModelMetric) {
    EXPECT_THROW(MapParser::parse("invalid_map.yaml"), std::runtime_error);
}

TEST_F(MapParserTest, ThrowsOnDuplicateZoneNames) {
    EXPECT_THROW(MapParser::parse("duplicate_zone_map.yaml"), std::runtime_error);
}

TEST_F(MapParserTest, ThrowsOnDeprecatedZoneId) {
    EXPECT_THROW(MapParser::parse("legacy_id_map.yaml"), std::runtime_error);
}

TEST_F(MapParserTest, ConvertsLatLonToWorld) {
    const auto zones = MapParser::parse("valid_lat_lon_map.yaml");
    ASSERT_EQ(zones.size(), 1u);

    const auto polygon = zones[0]->getPolygon();
    ASSERT_EQ(polygon.outer().size(), 2u);
    EXPECT_NEAR(polygon.outer()[0].x(), 0.0, 1e-6);
    EXPECT_NEAR(polygon.outer()[0].y(), 0.0, 1e-6);
    EXPECT_EQ(zones[0]->getPlannerBehavior().value_or(""), "parking_profile");
    EXPECT_EQ(zones[0]->getResolvedPlannerBehavior(), "parking_profile");
    EXPECT_GT(polygon.outer()[1].y(), 0.0);
    EXPECT_NEAR(polygon.outer()[1].x(), 0.0, 1e-3);
}

TEST_F(MapParserTest, KeepsWorldCoordinatesWithoutGeographicOrigin) {
    const auto zones = MapParser::parse("valid_world_map.yaml");
    ASSERT_EQ(zones.size(), 2u);

    const auto polygon = zones[0]->getPolygon();
    ASSERT_EQ(polygon.outer().size(), 2u);
    EXPECT_DOUBLE_EQ(polygon.outer()[0].x(), 1.0);
    EXPECT_DOUBLE_EQ(polygon.outer()[0].y(), 2.0);
    EXPECT_DOUBLE_EQ(polygon.outer()[1].x(), 3.0);
    EXPECT_DOUBLE_EQ(polygon.outer()[1].y(), 4.0);
    EXPECT_EQ(zones[0]->getPlannerBehavior().value_or(""), "primary_profile");
    EXPECT_EQ(zones[0]->getResolvedPlannerBehavior(), "primary_profile");
}

TEST_F(MapParserTest, ParsesSegmentedTrackRoadZone) {
    const auto zones = MapParser::parse("valid_world_map.yaml");
    ASSERT_EQ(zones.size(), 2u);

    const auto track =
        std::dynamic_pointer_cast<coastmotionplanning::zones::TrackMainRoad>(zones[1]);
    ASSERT_NE(track, nullptr);
    ASSERT_EQ(track->getCenterline().size(), 2u);
    ASSERT_EQ(track->getOffsets().size(), 2u);
    ASSERT_EQ(track->getLanes().size(), 2u);
    ASSERT_EQ(track->getLanes()[0].size(), 2u);
    ASSERT_EQ(track->getLanes()[1].size(), 2u);
    EXPECT_DOUBLE_EQ(track->getCenterline()[0].x, 1.0);
    EXPECT_DOUBLE_EQ(track->getCenterline()[0].y, 3.0);
    EXPECT_DOUBLE_EQ(track->getOffsets()[0], 1.0);
    EXPECT_DOUBLE_EQ(track->getLanes()[0][0].x, 1.0);
    EXPECT_DOUBLE_EQ(track->getLanes()[0][0].y, 2.0);
    EXPECT_DOUBLE_EQ(track->getLanes()[1][0].x, 9.0);
    EXPECT_DOUBLE_EQ(track->getLanes()[1][0].y, 4.0);
}

TEST_F(MapParserTest, AppliesModelMetricToLatLonConversion) {
    const auto base_zones = MapParser::parse("valid_lat_lon_map.yaml");
    const auto scaled_zones = MapParser::parse("scaled_lat_lon_map.yaml");

    ASSERT_EQ(base_zones.size(), 1u);
    ASSERT_EQ(scaled_zones.size(), 1u);

    const auto& base_polygon = base_zones[0]->getPolygon();
    const auto& scaled_polygon = scaled_zones[0]->getPolygon();
    ASSERT_EQ(base_polygon.outer().size(), 2u);
    ASSERT_EQ(scaled_polygon.outer().size(), 2u);

    EXPECT_NEAR(scaled_polygon.outer()[1].x(), 2.0 * base_polygon.outer()[1].x(), 1e-3);
    EXPECT_NEAR(scaled_polygon.outer()[1].y(), 2.0 * base_polygon.outer()[1].y(), 1e-3);
}

TEST_F(MapParserTest, ConvertsLongLatToWorld) {
    const auto zones = MapParser::parse("valid_long_lat_map.yaml");
    ASSERT_EQ(zones.size(), 1u);

    const auto polygon = zones[0]->getPolygon();
    ASSERT_EQ(polygon.outer().size(), 2u);
    EXPECT_NEAR(polygon.outer()[0].x(), 0.0, 1e-6);
    EXPECT_NEAR(polygon.outer()[0].y(), 0.0, 1e-6);
    EXPECT_GT(polygon.outer()[1].y(), 0.0);
    EXPECT_NEAR(polygon.outer()[1].x(), 0.0, 1e-3);
}

TEST_F(MapParserTest, ThrowsOnFileMissing) {
    EXPECT_THROW(MapParser::parse("non_existent.yaml"), std::runtime_error);
}

TEST_F(MapParserTest, LeavesBehaviorUnsetWhenBehaviorIsDefaultOrMissing) {
    const auto zones = MapParser::parse("default_behavior_map.yaml");
    ASSERT_EQ(zones.size(), 2u);

    EXPECT_FALSE(zones[0]->hasExplicitPlannerBehavior());
    EXPECT_EQ(zones[0]->getPlannerBehavior(), std::nullopt);
    EXPECT_TRUE(zones[0]->getResolvedPlannerBehavior().empty());

    EXPECT_FALSE(zones[1]->hasExplicitPlannerBehavior());
    EXPECT_EQ(zones[1]->getPlannerBehavior(), std::nullopt);
    EXPECT_TRUE(zones[1]->getResolvedPlannerBehavior().empty());
}

TEST_F(MapParserTest, ParsesMapYamlFromMemory) {
    const std::string yaml = R"(
maps:
  name: "Inline Map"
zones:
  - name: "inline_zone"
    type: "TrackMainRoad"
    coordinate_type: "world"
    polygon:
      - [0.0, -2.0]
      - [8.0, -2.0]
      - [8.0, 2.0]
      - [0.0, 2.0]
    lane:
      segments:
        - id: inline_center
          offset: 0.5
          center_waypoints:
            - [1.0, 0.5]
            - [7.0, 0.5]
)";

    const auto zones = MapParser::parseFromString(yaml, "inline-map");
    ASSERT_EQ(zones.size(), 1u);

    const auto track =
        std::dynamic_pointer_cast<coastmotionplanning::zones::TrackMainRoad>(zones[0]);
    ASSERT_NE(track, nullptr);
    EXPECT_TRUE(track->getResolvedPlannerBehavior().empty());
    ASSERT_EQ(track->getLanes().size(), 2u);
    ASSERT_EQ(track->getLanes()[0].size(), 2u);
    EXPECT_DOUBLE_EQ(track->getLanes()[0][0].x, 1.0);
    EXPECT_DOUBLE_EQ(track->getLanes()[0][0].y, 0.0);
}

TEST_F(MapParserTest, RejectsTrackRoadWithDeprecatedLanesField) {
    const std::string yaml = R"(
maps:
  name: "Deprecated Lanes"
zones:
  - name: "track"
    type: "TrackMainRoad"
    coordinate_type: "world"
    polygon:
      - [0.0, -2.0]
      - [8.0, -2.0]
      - [8.0, 2.0]
      - [0.0, 2.0]
    lanes:
      - lane_waypoints:
          - [1.0, -0.5]
          - [7.0, -0.5]
)";

    EXPECT_THROW(MapParser::parseFromString(yaml, "deprecated-lanes-map"), std::runtime_error);
}

TEST_F(MapParserTest, RejectsTrackRoadWithDeprecatedFlatLaneFields) {
    const std::string yaml = R"(
maps:
  name: "Deprecated Flat Lane"
zones:
  - name: "track"
    type: "TrackMainRoad"
    coordinate_type: "world"
    polygon:
      - [0.0, -2.0]
      - [8.0, -2.0]
      - [8.0, 2.0]
      - [0.0, 2.0]
    lane:
      center_waypoints:
        - [1.0, 0.0]
        - [7.0, 0.0]
      offsets:
        - 0.5
        - 0.5
)";

    EXPECT_THROW(MapParser::parseFromString(yaml, "deprecated-flat-map"), std::runtime_error);
}

TEST_F(MapParserTest, RejectsTrackRoadWithDuplicateSegmentIds) {
    const std::string yaml = R"(
maps:
  name: "Duplicate Segment Ids"
zones:
  - name: "track"
    type: "TrackMainRoad"
    coordinate_type: "world"
    polygon:
      - [0.0, -2.0]
      - [8.0, -2.0]
      - [8.0, 2.0]
      - [0.0, 2.0]
    lane:
      segments:
        - id: repeated
          offset: 0.5
          center_waypoints:
            - [1.0, 0.0]
            - [4.0, 0.0]
        - id: repeated
          offset: 0.5
          center_waypoints:
            - [4.0, 0.0]
            - [7.0, 0.0]
)";

    EXPECT_THROW(MapParser::parseFromString(yaml, "duplicate-segment-id-map"), std::runtime_error);
}

TEST_F(MapParserTest, RejectsTrackRoadWithNonScalarSegmentId) {
    const std::string yaml = R"(
maps:
  name: "Non Scalar Segment Id"
zones:
  - name: "track"
    type: "TrackMainRoad"
    coordinate_type: "world"
    polygon:
      - [0.0, -2.0]
      - [8.0, -2.0]
      - [8.0, 2.0]
      - [0.0, 2.0]
    lane:
      segments:
        - id:
            nested: bad
          offset: 0.5
          center_waypoints:
            - [1.0, 0.0]
            - [7.0, 0.0]
)";

    EXPECT_THROW(MapParser::parseFromString(yaml, "non-scalar-segment-id-map"), std::runtime_error);
}

TEST_F(MapParserTest, RejectsTrackRoadWithNegativeOffset) {
    const std::string yaml = R"(
maps:
  name: "Negative Offset"
zones:
  - name: "track"
    type: "TrackMainRoad"
    coordinate_type: "world"
    polygon:
      - [0.0, -2.0]
      - [8.0, -2.0]
      - [8.0, 2.0]
      - [0.0, 2.0]
    lane:
      segments:
        - id: negative
          offset: -0.5
          center_waypoints:
            - [1.0, 0.0]
            - [7.0, 0.0]
)";

    EXPECT_THROW(MapParser::parseFromString(yaml, "negative-offset-map"), std::invalid_argument);
}

TEST_F(MapParserTest, RejectsTrackRoadWithCollapsedSinglePointSegment) {
    const std::string yaml = R"(
maps:
  name: "Collapsed Single Point"
zones:
  - name: "track"
    type: "TrackMainRoad"
    coordinate_type: "world"
    polygon:
      - [0.0, -2.0]
      - [8.0, -2.0]
      - [8.0, 2.0]
      - [0.0, 2.0]
    lane:
      segments:
        - id: duplicate_points
          offset: 0.5
          center_waypoints:
            - [1.0, 0.0]
            - [1.0, 0.0]
)";

    EXPECT_THROW(MapParser::parseFromString(yaml, "collapsed-single-point-map"), std::invalid_argument);
}

TEST_F(MapParserTest, RejectsTrackRoadWhenBridgeLeavesPolygon) {
    const std::string yaml = R"(
maps:
  name: "Bridge Leaves Polygon"
zones:
  - name: "track"
    type: "TrackMainRoad"
    coordinate_type: "world"
    polygon:
      - [0.0, 0.0]
      - [8.0, 0.0]
      - [8.0, 2.0]
      - [2.0, 2.0]
      - [2.0, 8.0]
      - [0.0, 8.0]
    lane:
      segments:
        - id: horizontal
          offset: 0.25
          center_waypoints:
            - [1.0, 1.0]
            - [7.0, 1.0]
        - id: vertical
          offset: 0.25
          center_waypoints:
            - [1.0, 7.0]
            - [1.0, 3.0]
)";

    EXPECT_THROW(MapParser::parseFromString(yaml, "bridge-leaves-polygon-map"), std::invalid_argument);
}

TEST_F(MapParserTest, RejectsTrackRoadWhenDerivedLaneLeavesPolygon) {
    const std::string yaml = R"(
maps:
  name: "Derived Lane Outside"
zones:
  - name: "track"
    type: "TrackMainRoad"
    coordinate_type: "world"
    polygon:
      - [0.0, -2.0]
      - [8.0, -2.0]
      - [8.0, 2.0]
      - [0.0, 2.0]
    lane:
      segments:
        - id: high_centerline
          offset: 0.5
          center_waypoints:
            - [1.0, 1.75]
            - [7.0, 1.75]
)";

    EXPECT_THROW(MapParser::parseFromString(yaml, "derived-lane-outside-map"), std::invalid_argument);
}

TEST(TrackMainRoadValidationTest, BuildsDerivedOppositeDirectionLanesFromCenterline) {
    coastmotionplanning::geometry::Polygon2d polygon;
    polygon.outer() = {
        coastmotionplanning::geometry::Point2d(0.0, 0.0),
        coastmotionplanning::geometry::Point2d(10.0, 0.0),
        coastmotionplanning::geometry::Point2d(10.0, 6.0),
        coastmotionplanning::geometry::Point2d(0.0, 6.0),
        coastmotionplanning::geometry::Point2d(0.0, 0.0)
    };

    coastmotionplanning::zones::TrackMainRoad track(polygon, "track");
    track.setCenterlineFromPoints({
        coastmotionplanning::geometry::Point2d(1.0, 1.5),
        coastmotionplanning::geometry::Point2d(9.0, 1.5)
    }, {1.0, 1.0});

    ASSERT_EQ(track.getCenterline().size(), 2u);
    ASSERT_EQ(track.getLanes().size(), 2u);
    EXPECT_DOUBLE_EQ(track.getLanes()[0][0].x, 1.0);
    EXPECT_DOUBLE_EQ(track.getLanes()[0][0].y, 0.5);
    EXPECT_DOUBLE_EQ(track.getLanes()[1][0].x, 9.0);
    EXPECT_DOUBLE_EQ(track.getLanes()[1][0].y, 2.5);
    EXPECT_NEAR(track.getLanes()[0][0].theta.degrees(), 0.0, 1e-6);
    EXPECT_NEAR(std::abs(track.getLanes()[1][0].theta.degrees()), 180.0, 1e-6);
}

TEST(TrackMainRoadValidationTest, StitchesSeparatedSegmentsIntoCanonicalCenterline) {
    coastmotionplanning::geometry::Polygon2d polygon;
    polygon.outer() = {
        coastmotionplanning::geometry::Point2d(0.0, 0.0),
        coastmotionplanning::geometry::Point2d(10.0, 0.0),
        coastmotionplanning::geometry::Point2d(10.0, 6.0),
        coastmotionplanning::geometry::Point2d(0.0, 6.0),
        coastmotionplanning::geometry::Point2d(0.0, 0.0)
    };

    coastmotionplanning::zones::TrackMainRoad track(polygon, "track");
    track.setCenterlineSegments({
        {"segment_a", 1.0, {
            coastmotionplanning::geometry::Point2d(1.0, 3.0),
            coastmotionplanning::geometry::Point2d(4.0, 3.0)
        }},
        {"segment_b", 1.0, {
            coastmotionplanning::geometry::Point2d(6.0, 3.0),
            coastmotionplanning::geometry::Point2d(9.0, 3.0)
        }}
    });

    ASSERT_EQ(track.getCenterline().size(), 4u);
    EXPECT_DOUBLE_EQ(track.getCenterline()[1].x, 4.0);
    EXPECT_DOUBLE_EQ(track.getCenterline()[2].x, 6.0);
    EXPECT_DOUBLE_EQ(track.getOffsets()[0], 1.0);
    EXPECT_DOUBLE_EQ(track.getOffsets()[3], 1.0);
}

TEST(TrackMainRoadValidationTest, TouchingSegmentsWithDifferentOffsetsTaperAcrossNextSpan) {
    coastmotionplanning::geometry::Polygon2d polygon;
    polygon.outer() = {
        coastmotionplanning::geometry::Point2d(0.0, 0.0),
        coastmotionplanning::geometry::Point2d(10.0, 0.0),
        coastmotionplanning::geometry::Point2d(10.0, 8.0),
        coastmotionplanning::geometry::Point2d(0.0, 8.0),
        coastmotionplanning::geometry::Point2d(0.0, 0.0)
    };

    coastmotionplanning::zones::TrackMainRoad track(polygon, "track");
    track.setCenterlineSegments({
        {"segment_a", 1.0, {
            coastmotionplanning::geometry::Point2d(1.0, 4.0),
            coastmotionplanning::geometry::Point2d(5.0, 4.0)
        }},
        {"segment_b", 2.0, {
            coastmotionplanning::geometry::Point2d(5.0, 4.0),
            coastmotionplanning::geometry::Point2d(9.0, 4.0)
        }}
    });

    ASSERT_EQ(track.getCenterline().size(), 3u);
    ASSERT_EQ(track.getOffsets().size(), 3u);
    EXPECT_DOUBLE_EQ(track.getOffsets()[0], 1.0);
    EXPECT_DOUBLE_EQ(track.getOffsets()[1], 1.0);
    EXPECT_DOUBLE_EQ(track.getOffsets()[2], 2.0);
    EXPECT_DOUBLE_EQ(track.getLanes()[0][1].y, 3.0);
    EXPECT_DOUBLE_EQ(track.getLanes()[0][2].y, 2.0);
}

TEST(TrackMainRoadValidationTest, CollapsesNearDuplicateJoinNoise) {
    coastmotionplanning::geometry::Polygon2d polygon;
    polygon.outer() = {
        coastmotionplanning::geometry::Point2d(0.0, 0.0),
        coastmotionplanning::geometry::Point2d(10.0, 0.0),
        coastmotionplanning::geometry::Point2d(10.0, 6.0),
        coastmotionplanning::geometry::Point2d(0.0, 6.0),
        coastmotionplanning::geometry::Point2d(0.0, 0.0)
    };

    coastmotionplanning::zones::TrackMainRoad track(polygon, "track");
    track.setCenterlineSegments({
        {"segment_a", 1.0, {
            coastmotionplanning::geometry::Point2d(1.0, 3.0),
            coastmotionplanning::geometry::Point2d(5.0, 3.0)
        }},
        {"segment_b", 1.0, {
            coastmotionplanning::geometry::Point2d(5.0 + 5e-7, 3.0),
            coastmotionplanning::geometry::Point2d(9.0, 3.0)
        }}
    });

    ASSERT_EQ(track.getCenterline().size(), 3u);
    EXPECT_DOUBLE_EQ(track.getCenterline()[1].x, 5.0);
    EXPECT_DOUBLE_EQ(track.getCenterline()[2].x, 9.0);
}

TEST(TrackMainRoadValidationTest, RejectsOutOfOrderSegments) {
    coastmotionplanning::geometry::Polygon2d polygon;
    polygon.outer() = {
        coastmotionplanning::geometry::Point2d(0.0, 0.0),
        coastmotionplanning::geometry::Point2d(10.0, 0.0),
        coastmotionplanning::geometry::Point2d(10.0, 6.0),
        coastmotionplanning::geometry::Point2d(0.0, 6.0),
        coastmotionplanning::geometry::Point2d(0.0, 0.0)
    };

    coastmotionplanning::zones::TrackMainRoad track(polygon, "track");
    const std::vector<coastmotionplanning::zones::TrackMainRoadSegment> segments = {
        {"segment_a", 1.0, {
            coastmotionplanning::geometry::Point2d(1.0, 3.0),
            coastmotionplanning::geometry::Point2d(5.0, 3.0)
        }},
        {"segment_b", 1.0, {
            coastmotionplanning::geometry::Point2d(5.0, 3.0),
            coastmotionplanning::geometry::Point2d(3.0, 3.0)
        }}
    };
    EXPECT_THROW(
        track.setCenterlineSegments(segments),
        std::invalid_argument);
}
