#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "coastmotionplanning/planning/planner_behavior_profile.hpp"

namespace coastmotionplanning {
namespace planning {

class PlannerBehaviorSet {
public:
    PlannerBehaviorSet() = default;

    static PlannerBehaviorSet loadFromFile(const std::string& filepath);

    bool contains(const std::string& behavior_name) const;
    const PlannerBehaviorProfile& get(const std::string& behavior_name) const;
    const ZoneTypeTransitionPolicy* findTransitionPolicy(
        const std::string& from_zone_type,
        const std::string& to_zone_type) const;
    const std::vector<ZoneTypeTransitionPolicy>& transitionPolicies() const {
        return transition_policies_;
    }
    const PlannerBehaviorGlobalConfig& globalConfig() const { return global_config_; }
    bool debugModeEnabled() const { return global_config_.debug_mode; }
    void overrideMotionPrimitiveConstraints(double min_turning_radius_m,
                                            double max_steer_angle_rad);
    void setMinimumPlanningTimeMs(int minimum_planning_time_ms);
    const std::vector<std::string>& names() const { return names_; }

private:
    PlannerBehaviorGlobalConfig global_config_;
    std::vector<std::string> names_;
    std::unordered_map<std::string, PlannerBehaviorProfile> profiles_;
    std::vector<ZoneTypeTransitionPolicy> transition_policies_;
    std::unordered_map<std::string, size_t> transition_policy_indices_;
};

} // namespace planning
} // namespace coastmotionplanning
