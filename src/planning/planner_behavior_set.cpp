#include "coastmotionplanning/planning/planner_behavior_set.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>

#include "coastmotionplanning/config/planner_behavior_parser.hpp"

namespace coastmotionplanning {
namespace planning {

PlannerBehaviorSet PlannerBehaviorSet::loadFromFile(const std::string& filepath) {
    const std::filesystem::path behaviors_path(filepath);
    const std::filesystem::path master_params_path =
        behaviors_path.parent_path() / "master_params.yaml";
    const auto profiles = config::PlannerBehaviorParser::parse(
        master_params_path.string(), filepath);

    PlannerBehaviorSet behavior_set;
    for (const auto& entry : profiles) {
        behavior_set.names_.push_back(entry.first);
        behavior_set.profiles_.emplace(entry.first, entry.second);
    }
    std::sort(behavior_set.names_.begin(), behavior_set.names_.end());

    if (behavior_set.names_.empty()) {
        throw std::runtime_error(
            "Planner behavior file does not define any named behaviors: " + filepath);
    }

    return behavior_set;
}

bool PlannerBehaviorSet::contains(const std::string& behavior_name) const {
    return profiles_.find(behavior_name) != profiles_.end();
}

const PlannerBehaviorProfile& PlannerBehaviorSet::get(const std::string& behavior_name) const {
    const auto it = profiles_.find(behavior_name);
    if (it == profiles_.end()) {
        throw std::runtime_error("Planner behavior '" + behavior_name + "' is not defined.");
    }
    return it->second;
}

void PlannerBehaviorSet::overrideMotionPrimitiveConstraints(double min_turning_radius_m,
                                                            double max_steer_angle_rad) {
    if (min_turning_radius_m <= 0.0 || max_steer_angle_rad <= 0.0) {
        throw std::runtime_error("Motion primitive constraint overrides must be positive.");
    }

    for (auto& entry : profiles_) {
        entry.second.motion_primitives.min_turning_radius_m = min_turning_radius_m;
        entry.second.motion_primitives.max_steer_angle_rad = max_steer_angle_rad;
    }
}

void PlannerBehaviorSet::setMinimumPlanningTimeMs(int minimum_planning_time_ms) {
    if (minimum_planning_time_ms <= 0) {
        throw std::runtime_error("Minimum planning time must be positive.");
    }

    for (auto& entry : profiles_) {
        entry.second.planner.max_planning_time_ms =
            std::max(entry.second.planner.max_planning_time_ms, minimum_planning_time_ms);
    }
}

} // namespace planning
} // namespace coastmotionplanning
