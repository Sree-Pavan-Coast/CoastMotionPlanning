#include "coastmotionplanning/config/robots_parser.hpp"

#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace coastmotionplanning {
namespace config {

double CarDefinition::maxSteerAngleRadians() const {
    return max_steer_angle_deg * M_PI / 180.0;
}

double CarDefinition::minTurningRadiusMeters() const {
    const double steer_rad = maxSteerAngleRadians();
    if (!std::isfinite(steer_rad) || std::abs(std::tan(steer_rad)) < 1e-9) {
        throw std::runtime_error(
            "Car definition '" + name + "' has an invalid max steering angle.");
    }
    return wheelbase_m / std::tan(steer_rad);
}

namespace {

void requireMapNode(const YAML::Node& node, const std::string& context) {
    if (!node || !node.IsMap()) {
        throw std::runtime_error(context + " must be a YAML map.");
    }
}

std::string requireStringField(const YAML::Node& node,
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

void requireScalarFieldsLocal(const YAML::Node& node,
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

YAML::Node loadConfig(const std::string& filepath) {
    try {
        return YAML::LoadFile(filepath);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load robot YAML file: " + filepath + ". Error: " + e.what());
    }
}

const YAML::Node requireRobotsNode(const YAML::Node& config, const std::string& filepath) {
    const YAML::Node robots_node = config["robots"];
    if (!robots_node || !robots_node.IsSequence()) {
        throw std::runtime_error("Robot file missing 'robots' sequence: " + filepath);
    }
    return robots_node;
}

void validateRobotNode(const YAML::Node& robot_node,
                       const std::string& filepath,
                       std::unordered_set<std::string>& robot_names,
                       RobotDefinition& robot) {
    requireMapNode(robot_node, "Each robot entry");

    robot.name = requireStringField(robot_node, "name", "robot entry");
    if (robot_node["id"]) {
        throw std::runtime_error(
            "Robot '" + robot.name + "' uses deprecated 'id'. Use 'name' as the unique identifier.");
    }
    if (!robot_names.insert(robot.name).second) {
        throw std::runtime_error("Duplicate robot name '" + robot.name + "' found in: " + filepath);
    }

    robot.type = requireStringField(robot_node, "type", "robot '" + robot.name + "'");
}

CarDefinition makeCarDefinition(const YAML::Node& robot_node, const std::string& name) {
    requireMapNode(robot_node["geometry"], "geometry for robot '" + name + "'");
    requireMapNode(robot_node["kinematics"], "kinematics for robot '" + name + "'");
    requireScalarFieldsLocal(
        robot_node["geometry"],
        {"width_m", "wheelbase_m", "front_overhang_m", "rear_overhang_m"},
        "geometry for robot '" + name + "'");
    requireScalarFieldsLocal(
        robot_node["kinematics"],
        {"max_steer_angle_deg", "max_speed_mps", "max_reverse_speed_mps"},
        "kinematics for robot '" + name + "'");

    CarDefinition definition;
    definition.name = name;
    definition.width_m = robot_node["geometry"]["width_m"].as<double>();
    definition.wheelbase_m = robot_node["geometry"]["wheelbase_m"].as<double>();
    definition.front_overhang_m = robot_node["geometry"]["front_overhang_m"].as<double>();
    definition.rear_overhang_m = robot_node["geometry"]["rear_overhang_m"].as<double>();
    definition.max_steer_angle_deg = robot_node["kinematics"]["max_steer_angle_deg"].as<double>();
    definition.max_speed_mps = robot_node["kinematics"]["max_speed_mps"].as<double>();
    definition.max_reverse_speed_mps = robot_node["kinematics"]["max_reverse_speed_mps"].as<double>();
    return definition;
}

} // namespace

std::vector<RobotDefinition> RobotsParser::parse(const std::string& filepath) {
    const YAML::Node config = loadConfig(filepath);
    const YAML::Node robots_node = requireRobotsNode(config, filepath);

    std::unordered_set<std::string> robot_names;
    std::vector<RobotDefinition> robots;
    robots.reserve(robots_node.size());

    for (const auto& robot_node : robots_node) {
        RobotDefinition robot;
        validateRobotNode(robot_node, filepath, robot_names, robot);

        if (robot.type == "Car") {
            requireMapNode(robot_node["geometry"], "geometry for robot '" + robot.name + "'");
            requireMapNode(robot_node["kinematics"], "kinematics for robot '" + robot.name + "'");
            requireScalarFields(
                robot_node["geometry"],
                {"width_m", "wheelbase_m", "front_overhang_m", "rear_overhang_m"},
                "geometry for robot '" + robot.name + "'");
            requireScalarFields(
                robot_node["kinematics"],
                {"max_steer_angle_deg", "max_speed_mps", "max_reverse_speed_mps"},
                "kinematics for robot '" + robot.name + "'");
        } else if (robot.type == "TruckTrailer") {
            requireMapNode(robot_node["tractor"], "tractor for robot '" + robot.name + "'");
            requireMapNode(robot_node["trailer"], "trailer for robot '" + robot.name + "'");
            requireMapNode(robot_node["kinematics"], "kinematics for robot '" + robot.name + "'");
            requireScalarFields(
                robot_node["tractor"],
                {"width_m", "wheelbase_m", "front_overhang_m", "rear_overhang_m", "hitch_offset_m"},
                "tractor for robot '" + robot.name + "'");
            requireScalarFields(
                robot_node["trailer"],
                {"width_m", "wheelbase_m", "front_overhang_m", "rear_overhang_m"},
                "trailer for robot '" + robot.name + "'");
            requireScalarFields(
                robot_node["kinematics"],
                {"max_steer_angle_deg", "max_hitch_angle_deg", "max_speed_mps", "max_reverse_speed_mps"},
                "kinematics for robot '" + robot.name + "'");
        } else {
            throw std::runtime_error(
                "Unsupported robot type '" + robot.type + "' for robot '" + robot.name + "'.");
        }

        robots.push_back(std::move(robot));
    }

    return robots;
}

std::vector<CarDefinition> RobotsParser::parseCarDefinitions(const std::string& filepath) {
    const YAML::Node config = loadConfig(filepath);
    const YAML::Node robots_node = requireRobotsNode(config, filepath);

    std::unordered_set<std::string> robot_names;
    std::vector<CarDefinition> cars;

    for (const auto& robot_node : robots_node) {
        RobotDefinition robot;
        validateRobotNode(robot_node, filepath, robot_names, robot);

        if (robot.type != "Car") {
            continue;
        }

        cars.push_back(makeCarDefinition(robot_node, robot.name));
    }

    return cars;
}

CarDefinition RobotsParser::loadCarDefinition(const std::string& filepath,
                                              const std::string& robot_name) {
    const auto cars = parseCarDefinitions(filepath);
    for (const auto& car : cars) {
        if (car.name == robot_name) {
            return car;
        }
    }

    throw std::runtime_error(
        "Car robot '" + robot_name + "' was not found in: " + filepath);
}

robot::Car RobotsParser::loadCar(const std::string& filepath,
                                 const std::string& robot_name) {
    return loadCarDefinition(filepath, robot_name).buildCar();
}

std::string RobotsParser::requireString(const YAML::Node& node,
                                        const std::string& key,
                                        const std::string& context) {
    return requireStringField(node, key, context);
}

void RobotsParser::requireScalarFields(const YAML::Node& node,
                                       const std::vector<std::string>& keys,
                                       const std::string& context) {
    requireScalarFieldsLocal(node, keys, context);
}

} // namespace config
} // namespace coastmotionplanning
