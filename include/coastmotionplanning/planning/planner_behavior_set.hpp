#pragma once

#include <string>
#include <unordered_set>
#include <vector>

namespace coastmotionplanning {
namespace planning {

class PlannerBehaviorSet {
public:
    PlannerBehaviorSet() = default;

    static PlannerBehaviorSet loadFromFile(const std::string& filepath);

    bool contains(const std::string& behavior_name) const;
    const std::vector<std::string>& names() const { return names_; }

private:
    std::vector<std::string> names_;
    std::unordered_set<std::string> lookup_;
};

} // namespace planning
} // namespace coastmotionplanning
