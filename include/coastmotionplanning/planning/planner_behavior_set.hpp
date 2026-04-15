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
};

} // namespace planning
} // namespace coastmotionplanning
