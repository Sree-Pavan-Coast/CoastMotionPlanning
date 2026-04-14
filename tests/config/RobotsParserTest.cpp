#include <cstdio>
#include <fstream>

#include <gtest/gtest.h>

#include "coastmotionplanning/config/robots_parser.hpp"

using namespace coastmotionplanning::config;

class RobotsParserTest : public ::testing::Test {
protected:
    void TearDown() override {
        std::remove("valid_robots.yaml");
        std::remove("duplicate_robots.yaml");
        std::remove("legacy_robot_id.yaml");
    }
};

TEST_F(RobotsParserTest, ParsesNameOnlyRobotDefinitions) {
    std::ofstream ofs("valid_robots.yaml");
    ofs << "robots:\n"
        << "  - name: \"Dolly\"\n"
        << "    type: \"Car\"\n"
        << "    geometry:\n"
        << "      width_m: 3.022\n"
        << "      wheelbase_m: 3.37\n"
        << "      front_overhang_m: 0.55\n"
        << "      rear_overhang_m: 1.012\n"
        << "    kinematics:\n"
        << "      max_steer_angle_deg: 45.0\n"
        << "      max_speed_mps: 3.0\n"
        << "      max_reverse_speed_mps: 1.0\n";
    ofs.close();

    const auto robots = RobotsParser::parse("valid_robots.yaml");
    ASSERT_EQ(robots.size(), 1);
    EXPECT_EQ(robots.front().name, "Dolly");
    EXPECT_EQ(robots.front().type, "Car");
}

TEST_F(RobotsParserTest, ThrowsOnDuplicateRobotNames) {
    std::ofstream ofs("duplicate_robots.yaml");
    ofs << "robots:\n"
        << "  - name: \"Dolly\"\n"
        << "    type: \"Car\"\n"
        << "    geometry:\n"
        << "      width_m: 3.022\n"
        << "      wheelbase_m: 3.37\n"
        << "      front_overhang_m: 0.55\n"
        << "      rear_overhang_m: 1.012\n"
        << "    kinematics:\n"
        << "      max_steer_angle_deg: 45.0\n"
        << "      max_speed_mps: 3.0\n"
        << "      max_reverse_speed_mps: 1.0\n"
        << "  - name: \"Dolly\"\n"
        << "    type: \"Car\"\n"
        << "    geometry:\n"
        << "      width_m: 2.0\n"
        << "      wheelbase_m: 3.0\n"
        << "      front_overhang_m: 0.4\n"
        << "      rear_overhang_m: 0.6\n"
        << "    kinematics:\n"
        << "      max_steer_angle_deg: 30.0\n"
        << "      max_speed_mps: 4.0\n"
        << "      max_reverse_speed_mps: 1.0\n";
    ofs.close();

    EXPECT_THROW(RobotsParser::parse("duplicate_robots.yaml"), std::runtime_error);
}

TEST_F(RobotsParserTest, ThrowsOnDeprecatedRobotId) {
    std::ofstream ofs("legacy_robot_id.yaml");
    ofs << "robots:\n"
        << "  - id: \"dolly\"\n"
        << "    name: \"Dolly\"\n"
        << "    type: \"Car\"\n"
        << "    geometry:\n"
        << "      width_m: 3.022\n"
        << "      wheelbase_m: 3.37\n"
        << "      front_overhang_m: 0.55\n"
        << "      rear_overhang_m: 1.012\n"
        << "    kinematics:\n"
        << "      max_steer_angle_deg: 45.0\n"
        << "      max_speed_mps: 3.0\n"
        << "      max_reverse_speed_mps: 1.0\n";
    ofs.close();

    EXPECT_THROW(RobotsParser::parse("legacy_robot_id.yaml"), std::runtime_error);
}

TEST_F(RobotsParserTest, LoadsConcreteCarDefinitionByName) {
    std::ofstream ofs("valid_robots.yaml");
    ofs << "robots:\n"
        << "  - name: \"Dolly\"\n"
        << "    type: \"Car\"\n"
        << "    geometry:\n"
        << "      width_m: 3.022\n"
        << "      wheelbase_m: 3.37\n"
        << "      front_overhang_m: 0.55\n"
        << "      rear_overhang_m: 1.012\n"
        << "    kinematics:\n"
        << "      max_steer_angle_deg: 45.0\n"
        << "      max_speed_mps: 3.0\n"
        << "      max_reverse_speed_mps: 1.0\n";
    ofs.close();

    const auto definition = RobotsParser::loadCarDefinition("valid_robots.yaml", "Dolly");
    EXPECT_EQ(definition.name, "Dolly");
    EXPECT_DOUBLE_EQ(definition.width_m, 3.022);
    EXPECT_DOUBLE_EQ(definition.wheelbase_m, 3.37);
    EXPECT_DOUBLE_EQ(definition.max_steer_angle_deg, 45.0);
    EXPECT_NEAR(definition.maxSteerAngleRadians(), 0.78539816339, 1e-6);
    EXPECT_NEAR(definition.minTurningRadiusMeters(), 3.37, 1e-6);

    const auto car = RobotsParser::loadCar("valid_robots.yaml", "Dolly");
    EXPECT_DOUBLE_EQ(car.getWidth(), 3.022);
    EXPECT_DOUBLE_EQ(car.getWheelbase(), 3.37);
    EXPECT_DOUBLE_EQ(car.getFrontOverhang(), 0.55);
    EXPECT_DOUBLE_EQ(car.getRearOverhang(), 1.012);
}
