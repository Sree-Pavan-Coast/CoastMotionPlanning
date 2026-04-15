#pragma once

#include <string>
#include <unordered_set>

#include "coastmotionplanning/costs/costmap_types.hpp"

namespace coastmotionplanning {
namespace planning {

struct PlannerBehaviorGlobalConfig {
    bool debug_mode{false};
};

struct PlannerBehaviorPlannerConfig {
    int max_planning_time_ms{0};
    double xy_grid_resolution_m{0.0};
    double yaw_grid_resolution_deg{0.0};
    double step_size_m{0.0};
    bool only_forward_path{false};
    double weight_forward{0.0};
    double weight_reverse{0.0};
    double weight_steer{0.0};
    double weight_steer_change{0.0};
    double weight_gear_change{0.0};
    double analytic_expansion_max_length_m{0.0};
    double analytic_expansion_ratio{0.0};
    double min_path_len_in_same_motion{0.0};
    bool analytic_shot{true};
    bool near_goal_analytic_expansion{false};
    double near_goal_analytic_radius_m{0.0};
    double goal_approach_straight_distance_m{0.0};
    double weight_lane_centerline{1.0};
    double lane_heading_bias_weight{0.0};
    double max_cross_track_error_m{0.0};
    bool lane_primitive_suppression{false};
};

struct PlannerBehaviorCostmapConfig {
    double resolution_m{0.0};
    double inflation_radius_m{0.0};
    double inscribed_radius_m{0.0};
    double cost_scaling_factor{0.0};
    double alpha_shape_alpha{0.0};
    double max_lane_cost{0.0};
    double max_lane_half_width_m{0.0};

    costs::CostmapConfig toCostmapConfig() const;
};

struct PlannerBehaviorCollisionCheckerConfig {
    std::string collision_mode;
    double lethal_threshold{0.0};
    double margin_m{0.0};
};

struct PlannerBehaviorMotionPrimitivesConfig {
    int num_angle_bins{0};
    double min_turning_radius_m{0.0};
    double max_steer_angle_rad{0.0};
};

struct PlannerBehaviorNonHolonomicHeuristicConfig {
    int lut_grid_size{0};
    double lut_cell_size_m{0.0};
    double hitch_angle_penalty_factor{0.0};
};

struct PlannerBehaviorProfile {
    PlannerBehaviorPlannerConfig planner;
    PlannerBehaviorCostmapConfig costmap;
    PlannerBehaviorCollisionCheckerConfig collision_checker;
    PlannerBehaviorMotionPrimitivesConfig motion_primitives;
    PlannerBehaviorNonHolonomicHeuristicConfig non_holonomic_heuristic;
    std::unordered_set<std::string> active_layers;

    bool isLayerActive(const std::string& layer_name) const;
    costs::CostmapConfig makeCostmapConfig() const;
};

} // namespace planning
} // namespace coastmotionplanning
