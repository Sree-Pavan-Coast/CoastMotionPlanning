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

} // namespace planning
} // namespace coastmotionplanning
