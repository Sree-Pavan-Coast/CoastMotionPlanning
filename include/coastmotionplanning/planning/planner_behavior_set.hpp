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
    const std::vector<std::string>& names() const { return names_; }

private:
    std::vector<std::string> names_;
    std::unordered_map<std::string, PlannerBehaviorProfile> profiles_;
};

} // namespace planning
} // namespace coastmotionplanning
