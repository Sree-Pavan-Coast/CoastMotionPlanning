#pragma once

#include <string>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include "coastmotionplanning/planning/planner_behavior_profile.hpp"

namespace coastmotionplanning {
namespace config {

using PlannerBehaviorProfiles =
    std::unordered_map<std::string, planning::PlannerBehaviorProfile>;

struct PlannerBehaviorConfigFile {
    planning::PlannerBehaviorGlobalConfig global;
    PlannerBehaviorProfiles profiles;
};

class PlannerBehaviorParser {
public:
    static PlannerBehaviorConfigFile parse(const std::string& behaviors_filepath);

private:
    static planning::PlannerBehaviorGlobalConfig parseGlobalConfig(
        const YAML::Node& behaviors_root);
    static void validateProfileSchema(const YAML::Node& master_node,
                                      const YAML::Node& profile_node,
                                      const std::string& profile_name,
                                      const std::string& current_path);
    static void validateActiveLayers(const YAML::Node& active_layers_node,
                                     const std::string& profile_name);
    static planning::PlannerBehaviorProfile parseProfile(const YAML::Node& profile_node);
};

} // namespace config
} // namespace coastmotionplanning
