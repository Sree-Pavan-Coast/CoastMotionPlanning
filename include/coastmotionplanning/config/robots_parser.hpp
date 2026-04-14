#pragma once

#include <string>
#include <vector>

#include "coastmotionplanning/robot/car.hpp"

#include <yaml-cpp/yaml.h>

namespace coastmotionplanning {
namespace config {

struct RobotDefinition {
    std::string name;
    std::string type;
};

struct CarDefinition {
    std::string name;
    double width_m{0.0};
    double wheelbase_m{0.0};
    double front_overhang_m{0.0};
    double rear_overhang_m{0.0};
    double max_steer_angle_deg{0.0};
    double max_speed_mps{0.0};
    double max_reverse_speed_mps{0.0};

    robot::Car buildCar() const {
        return robot::Car(width_m, wheelbase_m, front_overhang_m, rear_overhang_m);
    }

    double maxSteerAngleRadians() const;
    double minTurningRadiusMeters() const;
};

class RobotsParser {
public:
    static std::vector<RobotDefinition> parse(const std::string& filepath);
    static std::vector<CarDefinition> parseCarDefinitions(const std::string& filepath);
    static CarDefinition loadCarDefinition(const std::string& filepath,
                                           const std::string& robot_name);
    static robot::Car loadCar(const std::string& filepath,
                              const std::string& robot_name);

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
