#include <gtest/gtest.h>
#include "coastmotionplanning/map/map_parser.hpp"
#include <fstream>

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
                  << "      - [3.0, 4.0]\n";
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

        // Create a temporary invalid YAML map (missing model_metric for geographic coordinates)
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
    }

    void TearDown() override {
        std::remove("valid_lat_lon_map.yaml");
        std::remove("valid_world_map.yaml");
        std::remove("scaled_lat_lon_map.yaml");
        std::remove("valid_long_lat_map.yaml");
        std::remove("invalid_map.yaml");
    }
};

TEST_F(MapParserTest, ThrowsOnMissingModelMetric) {
    EXPECT_THROW(MapParser::parse("invalid_map.yaml"), std::runtime_error);
}

TEST_F(MapParserTest, ConvertsLatLonToWorld) {
    auto zones = MapParser::parse("valid_lat_lon_map.yaml");
    ASSERT_EQ(zones.size(), 1);
    
    auto polygon = zones[0]->getPolygon();
    ASSERT_EQ(polygon.outer().size(), 2);

    // Origin point should be (0,0) in world if the model metric is 1.
    EXPECT_NEAR(polygon.outer()[0].x(), 0.0, 1e-6);
    EXPECT_NEAR(polygon.outer()[0].y(), 0.0, 1e-6);
    EXPECT_EQ(zones[0]->getPlannerBehavior().value_or(""), "parking_profile");

    // Point 2 is North of point 1, check if y is positive
    EXPECT_GT(polygon.outer()[1].y(), 0.0);
    EXPECT_NEAR(polygon.outer()[1].x(), 0.0, 1e-3); // Longitude is same, so x approx 0
}

TEST_F(MapParserTest, KeepsWorldCoordinatesWithoutGeographicOrigin) {
    auto zones = MapParser::parse("valid_world_map.yaml");
    ASSERT_EQ(zones.size(), 1);

    auto polygon = zones[0]->getPolygon();
    ASSERT_EQ(polygon.outer().size(), 2);
    EXPECT_DOUBLE_EQ(polygon.outer()[0].x(), 1.0);
    EXPECT_DOUBLE_EQ(polygon.outer()[0].y(), 2.0);
    EXPECT_DOUBLE_EQ(polygon.outer()[1].x(), 3.0);
    EXPECT_DOUBLE_EQ(polygon.outer()[1].y(), 4.0);
    EXPECT_EQ(zones[0]->getPlannerBehavior().value_or(""), "primary_profile");
}

TEST_F(MapParserTest, AppliesModelMetricToLatLonConversion) {
    auto base_zones = MapParser::parse("valid_lat_lon_map.yaml");
    auto scaled_zones = MapParser::parse("scaled_lat_lon_map.yaml");

    ASSERT_EQ(base_zones.size(), 1);
    ASSERT_EQ(scaled_zones.size(), 1);

    const auto& base_polygon = base_zones[0]->getPolygon();
    const auto& scaled_polygon = scaled_zones[0]->getPolygon();
    ASSERT_EQ(base_polygon.outer().size(), 2);
    ASSERT_EQ(scaled_polygon.outer().size(), 2);

    EXPECT_NEAR(scaled_polygon.outer()[1].x(), 2.0 * base_polygon.outer()[1].x(), 1e-3);
    EXPECT_NEAR(scaled_polygon.outer()[1].y(), 2.0 * base_polygon.outer()[1].y(), 1e-3);
}

TEST_F(MapParserTest, ConvertsLongLatToWorld) {
    auto zones = MapParser::parse("valid_long_lat_map.yaml");
    ASSERT_EQ(zones.size(), 1);

    auto polygon = zones[0]->getPolygon();
    ASSERT_EQ(polygon.outer().size(), 2);
    EXPECT_NEAR(polygon.outer()[0].x(), 0.0, 1e-6);
    EXPECT_NEAR(polygon.outer()[0].y(), 0.0, 1e-6);
    EXPECT_GT(polygon.outer()[1].y(), 0.0);
    EXPECT_NEAR(polygon.outer()[1].x(), 0.0, 1e-3);
}

TEST_F(MapParserTest, ThrowsOnFileMissing) {
    EXPECT_THROW(MapParser::parse("non_existent.yaml"), std::runtime_error);
}
