#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "coastmotionplanning/config/planner_behavior_parser.hpp"

using namespace coastmotionplanning::config;

namespace {

std::string schemaYaml() {
    return
        "planner:\n"
        "  max_planning_time_ms: 100\n"
        "  xy_grid_resolution_m: 0.1\n"
        "  yaw_grid_resolution_deg: 5.0\n"
        "  step_size_m: 0.5\n"
        "  only_forward_path: false\n"
        "  weight_forward: 1.0\n"
        "  weight_reverse: 2.0\n"
        "  weight_steer: 0.5\n"
        "  weight_steer_change: 1.5\n"
        "  weight_gear_change: 2.0\n"
        "  analytic_expansion_max_length_m: 20.0\n"
        "  analytic_expansion_ratio: 0.35\n"
        "  min_path_len_in_same_motion: 1.0\n"
        "  min_goal_straight_approach_m: 0.0\n"
        "  analytic_shot: true\n"
        "  near_goal_analytic_expansion: false\n"
        "  near_goal_analytic_radius_m: 0.0\n"
        "  weight_lane_centerline: 1.0\n"
        "  lane_heading_bias_weight: 0.0\n"
        "  max_cross_track_error_m: 0.0\n"
        "  lane_primitive_suppression: false\n"
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
        "non_holonomic_heuristic:\n"
        "  lut_grid_size: 400\n"
        "  lut_cell_size_m: 0.1\n"
        "  hitch_angle_penalty_factor: 2.0\n";
}

std::string indentYaml(const std::string& yaml, int spaces) {
    const std::string indent(static_cast<size_t>(spaces), ' ');
    std::string indented;
    size_t line_start = 0;
    while (line_start < yaml.size()) {
        size_t line_end = yaml.find('\n', line_start);
        if (line_end == std::string::npos) {
            line_end = yaml.size();
        }

        indented += indent;
        indented += yaml.substr(line_start, line_end - line_start);
        indented += '\n';
        line_start = line_end + 1;
    }
    return indented;
}

std::string schemaSectionYaml() {
    return "schema:\n" + indentYaml(schemaYaml(), 2);
}

std::string validBehaviorYaml() {
    return
        schemaSectionYaml() +
        "global:\n"
        "  debug_mode: true\n"
        "behaviors:\n"
        "  primary_profile:\n"
        "    planner:\n"
        "      max_planning_time_ms: 200\n"
        "      xy_grid_resolution_m: 0.1\n"
        "      yaw_grid_resolution_deg: 5.0\n"
        "      step_size_m: 0.5\n"
        "      only_forward_path: false\n"
        "      weight_forward: 1.0\n"
        "      weight_reverse: 2.5\n"
        "      weight_steer: 0.5\n"
        "      weight_steer_change: 1.5\n"
        "      weight_gear_change: 4.0\n"
        "      analytic_expansion_max_length_m: 20.0\n"
        "      analytic_expansion_ratio: 0.35\n"
        "      min_path_len_in_same_motion: 1.0\n"
        "      min_goal_straight_approach_m: 2.5\n"
        "      analytic_shot: true\n"
        "      near_goal_analytic_expansion: false\n"
        "      near_goal_analytic_radius_m: 0.0\n"
        "      weight_lane_centerline: 1.0\n"
        "      lane_heading_bias_weight: 0.0\n"
        "      max_cross_track_error_m: 0.0\n"
        "      lane_primitive_suppression: false\n"
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
        "    non_holonomic_heuristic:\n"
        "      lut_grid_size: 400\n"
        "      lut_cell_size_m: 0.1\n"
        "      hitch_angle_penalty_factor: 2.0\n"
        "    active_layers:\n"
        "      - static_obstacles\n";
}

std::string missingBehaviorYaml() {
    return
        schemaSectionYaml() +
        "behaviors:\n"
        "  primary_profile:\n"
        "    planner:\n"
        "      max_planning_time_ms: 200\n"
        "      xy_grid_resolution_m: 0.1\n"
        "      step_size_m: 0.5\n"
        "      only_forward_path: false\n"
        "      weight_forward: 1.0\n"
        "      weight_reverse: 2.5\n"
        "      weight_steer: 0.5\n"
        "      weight_steer_change: 1.5\n"
        "      weight_gear_change: 4.0\n"
        "      analytic_expansion_max_length_m: 20.0\n"
        "      analytic_expansion_ratio: 0.35\n"
        "      min_path_len_in_same_motion: 1.0\n"
        "      min_goal_straight_approach_m: 0.0\n"
        "      analytic_shot: true\n"
        "      near_goal_analytic_expansion: false\n"
        "      near_goal_analytic_radius_m: 0.0\n"
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
        "    non_holonomic_heuristic:\n"
        "      lut_grid_size: 400\n"
        "      lut_cell_size_m: 0.1\n"
        "      hitch_angle_penalty_factor: 2.0\n"
        "    active_layers:\n"
        "      - static_obstacles\n";
}

std::string extraBehaviorYaml() {
    return
        schemaSectionYaml() +
        "behaviors:\n"
        "  primary_profile:\n"
        "    planner:\n"
        "      max_planning_time_ms: 200\n"
        "      xy_grid_resolution_m: 0.1\n"
        "      yaw_grid_resolution_deg: 5.0\n"
        "      step_size_m: 0.5\n"
        "      only_forward_path: false\n"
        "      weight_forward: 1.0\n"
        "      weight_reverse: 2.5\n"
        "      weight_steer: 0.5\n"
        "      weight_steer_change: 1.5\n"
        "      weight_gear_change: 4.0\n"
        "      analytic_expansion_max_length_m: 20.0\n"
        "      analytic_expansion_ratio: 0.35\n"
        "      min_path_len_in_same_motion: 1.0\n"
        "      min_goal_straight_approach_m: 0.0\n"
        "      analytic_shot: true\n"
        "      near_goal_analytic_expansion: false\n"
        "      near_goal_analytic_radius_m: 0.0\n"
        "      weight_lane_centerline: 1.0\n"
        "      lane_heading_bias_weight: 0.0\n"
        "      max_cross_track_error_m: 0.0\n"
        "      lane_primitive_suppression: false\n"
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
        std::remove("valid_behaviors.yaml");
        std::remove("missing_behavior_key.yaml");
        std::remove("extra_behavior_key.yaml");
    }
};

TEST_F(PlannerBehaviorParserTest, ParsesProfilesThatMatchEmbeddedSchema) {
    const auto config_file = PlannerBehaviorParser::parse("valid_behaviors.yaml");
    const auto& profiles = config_file.profiles;

    ASSERT_EQ(profiles.size(), 1);
    EXPECT_TRUE(config_file.global.debug_mode);
    EXPECT_EQ(profiles.count("primary_profile"), 1);
    EXPECT_EQ(profiles.at("primary_profile").planner.max_planning_time_ms, 200);
    EXPECT_FALSE(profiles.at("primary_profile").planner.only_forward_path);
    EXPECT_DOUBLE_EQ(profiles.at("primary_profile").planner.weight_gear_change, 4.0);
    EXPECT_DOUBLE_EQ(
        profiles.at("primary_profile").planner.min_path_len_in_same_motion,
        1.0);
    EXPECT_DOUBLE_EQ(
        profiles.at("primary_profile").planner.min_goal_straight_approach_m,
        2.5);
    EXPECT_TRUE(profiles.at("primary_profile").planner.analytic_shot);
    EXPECT_FALSE(
        profiles.at("primary_profile").planner.near_goal_analytic_expansion);
    EXPECT_DOUBLE_EQ(
        profiles.at("primary_profile").planner.near_goal_analytic_radius_m,
        0.0);
    EXPECT_DOUBLE_EQ(
        profiles.at("primary_profile").planner.weight_lane_centerline,
        1.0);
    EXPECT_FALSE(
        profiles.at("primary_profile").planner.lane_primitive_suppression);
}

TEST_F(PlannerBehaviorParserTest, DefaultsGlobalDebugModeToFalseWhenGlobalNodeIsMissing) {
    std::ofstream no_global("no_global_behaviors.yaml");
    no_global << schemaSectionYaml()
              << validBehaviorYaml().substr(validBehaviorYaml().find("behaviors:\n"));
    no_global.close();

    const auto config_file = PlannerBehaviorParser::parse("no_global_behaviors.yaml");

    EXPECT_FALSE(config_file.global.debug_mode);
    std::remove("no_global_behaviors.yaml");
}

TEST_F(PlannerBehaviorParserTest, ThrowsWhenBehaviorOmitsSchemaKey) {
    EXPECT_THROW(PlannerBehaviorParser::parse("missing_behavior_key.yaml"), std::runtime_error);
}

TEST_F(PlannerBehaviorParserTest, ThrowsWhenBehaviorAddsUnexpectedKey) {
    EXPECT_THROW(PlannerBehaviorParser::parse("extra_behavior_key.yaml"), std::runtime_error);
}

TEST_F(PlannerBehaviorParserTest, ThrowsWhenSchemaNodeIsMissing) {
    std::ofstream no_schema("no_schema_behaviors.yaml");
    no_schema << validBehaviorYaml().substr(validBehaviorYaml().find("global:\n"));
    no_schema.close();

    EXPECT_THROW(PlannerBehaviorParser::parse("no_schema_behaviors.yaml"), std::runtime_error);
    std::remove("no_schema_behaviors.yaml");
}
