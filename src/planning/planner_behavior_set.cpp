#include "coastmotionplanning/planning/planner_behavior_set.hpp"

#include <algorithm>
#include <stdexcept>

#include "coastmotionplanning/config/planner_behavior_parser.hpp"

namespace coastmotionplanning {
namespace planning {

namespace {

std::string transitionKey(const std::string& from_zone_type,
                          const std::string& to_zone_type) {
    return from_zone_type + "->" + to_zone_type;
}

} // namespace

PlannerBehaviorSet PlannerBehaviorSet::loadFromFile(const std::string& filepath) {
    const auto config_file = config::PlannerBehaviorParser::parse(filepath);

    PlannerBehaviorSet behavior_set;
    behavior_set.global_config_ = config_file.global;
    for (const auto& entry : config_file.profiles) {
        behavior_set.names_.push_back(entry.first);
        behavior_set.profiles_.emplace(entry.first, entry.second);
    }
    std::sort(behavior_set.names_.begin(), behavior_set.names_.end());
    behavior_set.transition_policies_ = config_file.transition_policies;
    for (size_t index = 0; index < behavior_set.transition_policies_.size(); ++index) {
        const auto& policy = behavior_set.transition_policies_[index];
        behavior_set.transition_policy_indices_.emplace(
            transitionKey(policy.from_zone_type, policy.to_zone_type),
            index);
    }

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

const ZoneTypeTransitionPolicy* PlannerBehaviorSet::findTransitionPolicy(
    const std::string& from_zone_type,
    const std::string& to_zone_type) const {
    const auto it = transition_policy_indices_.find(
        transitionKey(from_zone_type, to_zone_type));
    if (it == transition_policy_indices_.end()) {
        return nullptr;
    }
    return &transition_policies_[it->second];
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
