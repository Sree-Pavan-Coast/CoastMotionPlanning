#pragma once

#include <string>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

namespace coastmotionplanning {
namespace config {

using PlannerBehaviorProfiles = std::unordered_map<std::string, YAML::Node>;

class PlannerBehaviorParser {
public:
    static PlannerBehaviorProfiles parse(const std::string& master_params_filepath,
                                         const std::string& behaviors_filepath);

private:
    static void validateProfileSchema(const YAML::Node& master_node,
                                      const YAML::Node& profile_node,
                                      const std::string& profile_name,
                                      const std::string& current_path);
    static void validateActiveLayers(const YAML::Node& active_layers_node,
                                     const std::string& profile_name);
};

} // namespace config
} // namespace coastmotionplanning
