#include <cstdio>
#include <fstream>

#include <gtest/gtest.h>

#include "coastmotionplanning/config/planner_behavior_parser.hpp"

using namespace coastmotionplanning::config;

class PlannerBehaviorParserTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::ofstream master("test_master_params.yaml");
        master << "planner:\n"
               << "  max_planning_time_ms: 100\n"
               << "  step_size_m: 0.5\n"
               << "costmap:\n"
               << "  resolution_m: 0.1\n"
               << "collision_checker:\n"
               << "  collision_mode: \"strict\"\n";
        master.close();

        std::ofstream valid("valid_behaviors.yaml");
        valid << "behaviors:\n"
              << "  primary_profile:\n"
              << "    planner:\n"
              << "      max_planning_time_ms: 200\n"
              << "      step_size_m: 0.5\n"
              << "    costmap:\n"
              << "      resolution_m: 0.1\n"
              << "    collision_checker:\n"
              << "      collision_mode: \"strict\"\n"
              << "    active_layers:\n"
              << "      - static_obstacles\n";
        valid.close();

        std::ofstream missing("missing_behavior_key.yaml");
        missing << "behaviors:\n"
                << "  primary_profile:\n"
                << "    planner:\n"
                << "      max_planning_time_ms: 200\n"
                << "    costmap:\n"
                << "      resolution_m: 0.1\n"
                << "    collision_checker:\n"
                << "      collision_mode: \"strict\"\n"
                << "    active_layers:\n"
                << "      - static_obstacles\n";
        missing.close();

        std::ofstream extra("extra_behavior_key.yaml");
        extra << "behaviors:\n"
              << "  primary_profile:\n"
              << "    planner:\n"
              << "      max_planning_time_ms: 200\n"
              << "      step_size_m: 0.5\n"
              << "      unexpected_weight: 1.0\n"
              << "    costmap:\n"
              << "      resolution_m: 0.1\n"
              << "    collision_checker:\n"
              << "      collision_mode: \"strict\"\n"
              << "    active_layers:\n"
              << "      - static_obstacles\n";
        extra.close();
    }

    void TearDown() override {
        std::remove("test_master_params.yaml");
        std::remove("valid_behaviors.yaml");
        std::remove("missing_behavior_key.yaml");
        std::remove("extra_behavior_key.yaml");
    }
};

TEST_F(PlannerBehaviorParserTest, ParsesProfilesThatMatchMasterSchema) {
    const auto profiles =
        PlannerBehaviorParser::parse("test_master_params.yaml", "valid_behaviors.yaml");

    ASSERT_EQ(profiles.size(), 1);
    EXPECT_EQ(profiles.count("primary_profile"), 1);
}

TEST_F(PlannerBehaviorParserTest, ThrowsWhenBehaviorOmitsMasterKey) {
    EXPECT_THROW(
        PlannerBehaviorParser::parse("test_master_params.yaml", "missing_behavior_key.yaml"),
        std::runtime_error);
}

TEST_F(PlannerBehaviorParserTest, ThrowsWhenBehaviorAddsUnexpectedKey) {
    EXPECT_THROW(
        PlannerBehaviorParser::parse("test_master_params.yaml", "extra_behavior_key.yaml"),
        std::runtime_error);
}
