#pragma once

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace coastmotionplanning {
namespace config {

struct RobotDefinition {
    std::string name;
    std::string type;
};

class RobotsParser {
public:
    static std::vector<RobotDefinition> parse(const std::string& filepath);

private:
    static std::string requireString(const YAML::Node& node,
                                     const std::string& key,
                                     const std::string& context);
    static void requireScalarFields(const YAML::Node& node,
                                    const std::vector<std::string>& keys,
                                    const std::string& context);
};

} // namespace config
} // namespace coastmotionplanning
