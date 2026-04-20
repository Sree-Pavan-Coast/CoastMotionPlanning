#include "coastmotionplanning/planning/planner_behavior_profile.hpp"

namespace coastmotionplanning {
namespace planning {

costs::CostmapConfig PlannerBehaviorCostmapConfig::toCostmapConfig() const {
    costs::CostmapConfig config;
    config.resolution = resolution_m;
    config.inflation_radius_m = inflation_radius_m;
    config.inscribed_radius_m = inscribed_radius_m;
    config.cost_scaling_factor = cost_scaling_factor;
    config.max_lane_cost = max_lane_cost;
    config.max_lane_half_width = max_lane_half_width_m;
    return config;
}

bool PlannerBehaviorProfile::isLayerActive(const std::string& layer_name) const {
    return active_layers.find(layer_name) != active_layers.end();
}

costs::CostmapConfig PlannerBehaviorProfile::makeCostmapConfig() const {
    costs::CostmapConfig config = costmap.toCostmapConfig();
    config.hitch_angle_penalty_factor = non_holonomic_heuristic.hitch_angle_penalty_factor;
    return config;
}

} // namespace planning
} // namespace coastmotionplanning
