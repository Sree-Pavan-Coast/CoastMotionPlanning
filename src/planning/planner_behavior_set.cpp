#include "coastmotionplanning/planning/planner_behavior_set.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace coastmotionplanning {
namespace planning {

PlannerBehaviorSet PlannerBehaviorSet::loadFromFile(const std::string& filepath) {
    YAML::Node config;
    try {
        config = YAML::LoadFile(filepath);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load planner behavior file: " + filepath +
                                 ". Error: " + e.what());
    }

    const YAML::Node behaviors = config["behaviors"];
    if (!behaviors || !behaviors.IsMap()) {
        throw std::runtime_error(
            "Planner behavior file missing a 'behaviors' map: " + filepath);
    }

    PlannerBehaviorSet behavior_set;
    for (const auto& entry : behaviors) {
        const std::string behavior_name = entry.first.as<std::string>("");
        if (behavior_name.empty()) {
            continue;
        }
        if (behavior_set.lookup_.insert(behavior_name).second) {
            behavior_set.names_.push_back(behavior_name);
        }
    }

    if (behavior_set.names_.empty()) {
        throw std::runtime_error(
            "Planner behavior file does not define any named behaviors: " + filepath);
    }

    return behavior_set;
}

bool PlannerBehaviorSet::contains(const std::string& behavior_name) const {
    return lookup_.find(behavior_name) != lookup_.end();
}

} // namespace planning
} // namespace coastmotionplanning
