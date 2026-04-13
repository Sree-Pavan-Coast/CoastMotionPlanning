#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "coastmotionplanning/config/planner_behavior_parser.hpp"

using namespace coastmotionplanning::config;

namespace {

std::string masterYaml() {
    return
        "planner:\n"
        "  max_planning_time_ms: 100\n"
        "  xy_grid_resolution_m: 0.1\n"
        "  yaw_grid_resolution_deg: 5.0\n"
        "  step_size_m: 0.5\n"
        "  weight_forward: 1.0\n"
        "  weight_reverse: 2.0\n"
        "  weight_steer: 0.5\n"
        "  weight_steer_change: 1.5\n"
        "  analytic_expansion_max_length_m: 20.0\n"
        "  analytic_expansion_ratio: 0.35\n"
        "costmap:\n"
        "  resolution_m: 0.1\n"
        "  inflation_radius_m: 0.3\n"
        "  inscribed_radius_m: 1.0\n"
        "  cost_scaling_factor: 3.0\n"
        "  alpha_shape_alpha: 0.0\n"
        "  max_lane_cost: 100.0\n"
        "  max_lane_half_width_m: 5.0\n"
        "collision_checker:\n"
        "  collision_mode: \"strict\"\n"
        "  lethal_threshold: 254.0\n"
        "  margin_m: 0.0\n"
        "motion_primitives:\n"
        "  num_angle_bins: 72\n"
        "  min_turning_radius_m: 8.0\n"
        "  max_steer_angle_rad: 0.5\n"
        "non_holonomic_heuristic:\n"
        "  lut_grid_size: 400\n"
        "  lut_cell_size_m: 0.1\n"
        "  hitch_angle_penalty_factor: 2.0\n";
}

std::string validBehaviorYaml() {
    return
        "behaviors:\n"
        "  primary_profile:\n"
        "    planner:\n"
        "      max_planning_time_ms: 200\n"
        "      xy_grid_resolution_m: 0.1\n"
        "      yaw_grid_resolution_deg: 5.0\n"
        "      step_size_m: 0.5\n"
        "      weight_forward: 1.0\n"
        "      weight_reverse: 2.5\n"
        "      weight_steer: 0.5\n"
        "      weight_steer_change: 1.5\n"
        "      analytic_expansion_max_length_m: 20.0\n"
        "      analytic_expansion_ratio: 0.35\n"
        "    costmap:\n"
        "      resolution_m: 0.1\n"
        "      inflation_radius_m: 0.3\n"
        "      inscribed_radius_m: 1.0\n"
        "      cost_scaling_factor: 3.0\n"
        "      alpha_shape_alpha: 0.0\n"
        "      max_lane_cost: 100.0\n"
        "      max_lane_half_width_m: 5.0\n"
        "    collision_checker:\n"
        "      collision_mode: \"strict\"\n"
        "      lethal_threshold: 254.0\n"
        "      margin_m: 0.0\n"
        "    motion_primitives:\n"
        "      num_angle_bins: 72\n"
        "      min_turning_radius_m: 8.0\n"
        "      max_steer_angle_rad: 0.5\n"
        "    non_holonomic_heuristic:\n"
        "      lut_grid_size: 400\n"
        "      lut_cell_size_m: 0.1\n"
        "      hitch_angle_penalty_factor: 2.0\n"
        "    active_layers:\n"
        "      - static_obstacles\n";
}

std::string missingBehaviorYaml() {
    return
        "behaviors:\n"
        "  primary_profile:\n"
        "    planner:\n"
        "      max_planning_time_ms: 200\n"
        "      xy_grid_resolution_m: 0.1\n"
        "      step_size_m: 0.5\n"
        "      weight_forward: 1.0\n"
        "      weight_reverse: 2.5\n"
        "      weight_steer: 0.5\n"
        "      weight_steer_change: 1.5\n"
        "      analytic_expansion_max_length_m: 20.0\n"
        "      analytic_expansion_ratio: 0.35\n"
        "    costmap:\n"
        "      resolution_m: 0.1\n"
        "      inflation_radius_m: 0.3\n"
        "      inscribed_radius_m: 1.0\n"
        "      cost_scaling_factor: 3.0\n"
        "      alpha_shape_alpha: 0.0\n"
        "      max_lane_cost: 100.0\n"
        "      max_lane_half_width_m: 5.0\n"
        "    collision_checker:\n"
        "      collision_mode: \"strict\"\n"
        "      lethal_threshold: 254.0\n"
        "      margin_m: 0.0\n"
        "    motion_primitives:\n"
        "      num_angle_bins: 72\n"
        "      min_turning_radius_m: 8.0\n"
        "      max_steer_angle_rad: 0.5\n"
        "    non_holonomic_heuristic:\n"
        "      lut_grid_size: 400\n"
        "      lut_cell_size_m: 0.1\n"
        "      hitch_angle_penalty_factor: 2.0\n"
        "    active_layers:\n"
        "      - static_obstacles\n";
}

std::string extraBehaviorYaml() {
    return
        "behaviors:\n"
        "  primary_profile:\n"
        "    planner:\n"
        "      max_planning_time_ms: 200\n"
        "      xy_grid_resolution_m: 0.1\n"
        "      yaw_grid_resolution_deg: 5.0\n"
        "      step_size_m: 0.5\n"
        "      weight_forward: 1.0\n"
        "      weight_reverse: 2.5\n"
        "      weight_steer: 0.5\n"
        "      weight_steer_change: 1.5\n"
        "      analytic_expansion_max_length_m: 20.0\n"
        "      analytic_expansion_ratio: 0.35\n"
        "      unexpected_weight: 1.0\n"
        "    costmap:\n"
        "      resolution_m: 0.1\n"
        "      inflation_radius_m: 0.3\n"
        "      inscribed_radius_m: 1.0\n"
        "      cost_scaling_factor: 3.0\n"
        "      alpha_shape_alpha: 0.0\n"
        "      max_lane_cost: 100.0\n"
        "      max_lane_half_width_m: 5.0\n"
        "    collision_checker:\n"
        "      collision_mode: \"strict\"\n"
        "      lethal_threshold: 254.0\n"
        "      margin_m: 0.0\n"
        "    motion_primitives:\n"
        "      num_angle_bins: 72\n"
        "      min_turning_radius_m: 8.0\n"
        "      max_steer_angle_rad: 0.5\n"
        "    non_holonomic_heuristic:\n"
        "      lut_grid_size: 400\n"
        "      lut_cell_size_m: 0.1\n"
        "      hitch_angle_penalty_factor: 2.0\n"
        "    active_layers:\n"
        "      - static_obstacles\n";
}

} // namespace

class PlannerBehaviorParserTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::ofstream master("test_master_params.yaml");
        master << masterYaml();
        master.close();

        std::ofstream valid("valid_behaviors.yaml");
        valid << validBehaviorYaml();
        valid.close();

        std::ofstream missing("missing_behavior_key.yaml");
        missing << missingBehaviorYaml();
        missing.close();

        std::ofstream extra("extra_behavior_key.yaml");
        extra << extraBehaviorYaml();
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
    EXPECT_EQ(profiles.at("primary_profile").planner.max_planning_time_ms, 200);
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
