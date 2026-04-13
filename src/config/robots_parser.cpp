#include "coastmotionplanning/config/robots_parser.hpp"

#include <stdexcept>
#include <unordered_set>

namespace coastmotionplanning {
namespace config {

namespace {

void requireMapNode(const YAML::Node& node, const std::string& context) {
    if (!node || !node.IsMap()) {
        throw std::runtime_error(context + " must be a YAML map.");
    }
}

} // namespace

std::vector<RobotDefinition> RobotsParser::parse(const std::string& filepath) {
    YAML::Node config;
    try {
        config = YAML::LoadFile(filepath);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load robot YAML file: " + filepath + ". Error: " + e.what());
    }

    const YAML::Node robots_node = config["robots"];
    if (!robots_node || !robots_node.IsSequence()) {
        throw std::runtime_error("Robot file missing 'robots' sequence: " + filepath);
    }

    std::unordered_set<std::string> robot_names;
    std::vector<RobotDefinition> robots;
    robots.reserve(robots_node.size());

    for (const auto& robot_node : robots_node) {
        requireMapNode(robot_node, "Each robot entry");

        const std::string name = requireString(robot_node, "name", "robot entry");
        if (robot_node["id"]) {
            throw std::runtime_error(
                "Robot '" + name + "' uses deprecated 'id'. Use 'name' as the unique identifier.");
        }
        if (!robot_names.insert(name).second) {
            throw std::runtime_error("Duplicate robot name '" + name + "' found in: " + filepath);
        }

        const std::string type = requireString(robot_node, "type", "robot '" + name + "'");

        if (type == "Car") {
            requireMapNode(robot_node["geometry"], "geometry for robot '" + name + "'");
            requireMapNode(robot_node["kinematics"], "kinematics for robot '" + name + "'");
            requireScalarFields(
                robot_node["geometry"],
                {"width_m", "wheelbase_m", "front_overhang_m", "rear_overhang_m"},
                "geometry for robot '" + name + "'");
            requireScalarFields(
                robot_node["kinematics"],
                {"max_steer_angle_deg", "max_speed_mps", "max_reverse_speed_mps"},
                "kinematics for robot '" + name + "'");
        } else if (type == "TruckTrailer") {
            requireMapNode(robot_node["tractor"], "tractor for robot '" + name + "'");
            requireMapNode(robot_node["trailer"], "trailer for robot '" + name + "'");
            requireMapNode(robot_node["kinematics"], "kinematics for robot '" + name + "'");
            requireScalarFields(
                robot_node["tractor"],
                {"width_m", "wheelbase_m", "front_overhang_m", "rear_overhang_m", "hitch_offset_m"},
                "tractor for robot '" + name + "'");
            requireScalarFields(
                robot_node["trailer"],
                {"width_m", "wheelbase_m", "front_overhang_m", "rear_overhang_m"},
                "trailer for robot '" + name + "'");
            requireScalarFields(
                robot_node["kinematics"],
                {"max_steer_angle_deg", "max_hitch_angle_deg", "max_speed_mps", "max_reverse_speed_mps"},
                "kinematics for robot '" + name + "'");
        } else {
            throw std::runtime_error("Unsupported robot type '" + type + "' for robot '" + name + "'.");
        }

        robots.push_back({name, type});
    }

    return robots;
}

std::string RobotsParser::requireString(const YAML::Node& node,
                                        const std::string& key,
                                        const std::string& context) {
    const YAML::Node value_node = node[key];
    if (!value_node || !value_node.IsScalar()) {
        throw std::runtime_error(context + " missing required scalar '" + key + "'.");
    }

    const std::string value = value_node.as<std::string>("");
    if (value.empty()) {
        throw std::runtime_error(context + " has empty required field '" + key + "'.");
    }
    return value;
}

void RobotsParser::requireScalarFields(const YAML::Node& node,
                                       const std::vector<std::string>& keys,
                                       const std::string& context) {
    requireMapNode(node, context);
    for (const auto& key : keys) {
        const YAML::Node value_node = node[key];
        if (!value_node || !value_node.IsScalar()) {
            throw std::runtime_error(context + " missing required scalar '" + key + "'.");
        }
    }
}

} // namespace config
} // namespace coastmotionplanning
