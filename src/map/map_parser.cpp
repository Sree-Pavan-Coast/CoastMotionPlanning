#include "coastmotionplanning/map/map_parser.hpp"
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <unordered_set>

namespace coastmotionplanning {
namespace map {

namespace {

std::optional<std::string> parsePlannerBehaviorOverride(const YAML::Node& zone_node) {
    const std::string planner_behavior_name = zone_node["planner_behavior"].as<std::string>("");
    if (planner_behavior_name.empty() || planner_behavior_name == "default") {
        return std::nullopt;
    }
    return planner_behavior_name;
}

} // namespace

std::vector<std::shared_ptr<zones::Zone>> MapParser::parse(const std::string& filepath) {
    YAML::Node config;
    try {
        config = YAML::LoadFile(filepath);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load YAML file: " + filepath + ". Error: " + e.what());
    }

    return parseDocument(config, filepath);
}

std::vector<std::shared_ptr<zones::Zone>> MapParser::parseFromString(
    const std::string& yaml_content,
    const std::string& source_label) {
    YAML::Node config;
    try {
        config = YAML::Load(yaml_content);
    } catch (const std::exception& e) {
        throw std::runtime_error(
            "Failed to load YAML content: " + source_label + ". Error: " + e.what());
    }

    return parseDocument(config, source_label);
}

std::vector<std::shared_ptr<zones::Zone>> MapParser::parseDocument(
    const YAML::Node& config,
    const std::string& source_label) {

    if (!config["maps"]) {
        throw std::runtime_error("Map file missing 'maps' section: " + source_label);
    }

    const auto& maps_node = config["maps"];
    std::optional<CoordinateTransform::LLA> origin;
    std::optional<double> model_metric;

    std::vector<std::shared_ptr<zones::Zone>> zones_list;
    std::unordered_set<std::string> zone_names;

    if (config["zones"]) {
        for (const auto& zone_node : config["zones"]) {
            if (!zone_node.IsMap()) {
                throw std::runtime_error("Each zone entry must be a YAML map in: " + source_label);
            }

            const std::string name = zone_node["name"].as<std::string>("");
            if (name.empty()) {
                throw std::runtime_error(
                    "Zone entries must define a non-empty 'name' in: " + source_label);
            }
            if (zone_node["id"]) {
                throw std::runtime_error(
                    "Zone '" + name + "' uses deprecated 'id'. Use 'name' as the unique identifier.");
            }
            if (!zone_names.insert(name).second) {
                throw std::runtime_error(
                    "Duplicate zone name '" + name + "' found in: " + source_label);
            }

            std::string type = zone_node["type"].as<std::string>("");
            const std::string zone_label = name;
            const CoordinateType coordinate_type = parseCoordinateType(zone_node, zone_label);
            if (coordinate_type != CoordinateType::WORLD) {
                if (!origin.has_value()) {
                    origin = parseOrigin(maps_node);
                }
                if (!model_metric.has_value()) {
                    model_metric = parseModelMetric(maps_node);
                }
            }

            auto polygon_points = parsePoints(
                zone_node["polygon"],
                coordinate_type,
                origin,
                model_metric.value_or(1.0),
                "polygon for zone '" + zone_label + "'");
            geometry::Polygon2d polygon;
            for (const auto& p : polygon_points) {
                polygon.outer().push_back(p);
            }

            const std::optional<std::string> planner_behavior =
                parsePlannerBehaviorOverride(zone_node);

            if (type == "ManeuveringZone") {
                auto zone = std::make_shared<zones::ManeuveringZone>(polygon, name);
                zone->setPlannerBehavior(planner_behavior);
                zones_list.push_back(zone);
            } else if (type == "TrackMainRoad") {
                std::vector<std::vector<geometry::Point2d>> lane_point_sequences;
                if (zone_node["lanes"]) {
                    for (const auto& lane_node : zone_node["lanes"]) {
                        lane_point_sequences.push_back(parsePoints(
                            lane_node["lane_waypoints"],
                            coordinate_type,
                            origin,
                            model_metric.value_or(1.0),
                            "lane_waypoints for zone '" + zone_label + "'"));
                    }
                }
                auto track = std::make_shared<zones::TrackMainRoad>(
                    polygon, lane_point_sequences, name);
                track->setPlannerBehavior(planner_behavior);
                zones_list.push_back(track);
            } else {
                // Fallback to ManeuveringZone if type is unknown or missing
                std::cerr << "MapParser: unknown zone type '" << type 
                          << "' for zone '" << name << "', defaulting to ManeuveringZone" << std::endl;
                auto zone = std::make_shared<zones::ManeuveringZone>(polygon, name);
                zone->setPlannerBehavior(planner_behavior);
                zones_list.push_back(zone);
            }
        }
    }

    return zones_list;
}

CoordinateTransform::LLA MapParser::parseOrigin(const YAML::Node& maps_node) {
    if (!maps_node["geographic_origin"]) {
        throw std::runtime_error("Map missing 'geographic_origin' section.");
    }
    const auto& origin_node = maps_node["geographic_origin"];
    if (!origin_node["latitude"] || !origin_node["longitude"]) {
        throw std::runtime_error("geographic_origin missing latitude or longitude.");
    }

    CoordinateTransform::LLA lla;
    lla.lat = origin_node["latitude"].as<double>();
    lla.lon = origin_node["longitude"].as<double>();
    lla.alt = origin_node["altitude"].as<double>(0.0);
    return lla;
}

double MapParser::parseModelMetric(const YAML::Node& maps_node) {
    if (!maps_node["model_metric"]) {
        throw std::runtime_error("Map missing 'model_metric'.");
    }

    const double model_metric = maps_node["model_metric"].as<double>();
    if (!std::isfinite(model_metric) || model_metric <= 0.0) {
        throw std::runtime_error("model_metric must be a finite positive number.");
    }
    return model_metric;
}

MapParser::CoordinateType MapParser::parseCoordinateType(const YAML::Node& zone_node,
                                                         const std::string& zone_label) {
    const std::string coordinate_type = zone_node["coordinate_type"].as<std::string>("world");
    if (coordinate_type == "world") {
        return CoordinateType::WORLD;
    }
    if (coordinate_type == "lat_long") {
        return CoordinateType::LAT_LONG;
    }
    if (coordinate_type == "long_lat") {
        return CoordinateType::LONG_LAT;
    }

    throw std::runtime_error(
        "Unsupported coordinate_type '" + coordinate_type + "' for zone '" + zone_label +
        "'. Expected one of: world, lat_long, long_lat.");
}

std::vector<geometry::Point2d> MapParser::parsePoints(const YAML::Node& points_node, 
                                                      CoordinateType coordinate_type,
                                                      const std::optional<CoordinateTransform::LLA>& origin,
                                                      double model_metric,
                                                      const std::string& context) {
    std::vector<geometry::Point2d> points;
    if (!points_node || !points_node.IsSequence()) {
        return points;
    }

    for (const auto& node : points_node) {
        if (!node.IsSequence()) {
            throw std::runtime_error("Expected coordinate pair entries in " + context + ".");
        }
        if (node.size() != 2) {
            throw std::runtime_error("Expected 2-value coordinate pairs in " + context + ".");
        }

        double x, y;
        if (coordinate_type == CoordinateType::WORLD) {
            x = node[0].as<double>();
            y = node[1].as<double>();
        } else {
            if (!origin.has_value()) {
                throw std::runtime_error("geographic_origin is required for " + context + ".");
            }

            CoordinateTransform::LLA lla;
            if (coordinate_type == CoordinateType::LAT_LONG) {
                lla.lat = node[0].as<double>();
                lla.lon = node[1].as<double>();
            } else {
                lla.lon = node[0].as<double>();
                lla.lat = node[1].as<double>();
            }
            lla.alt = 0.0;

            Eigen::Vector3d world = CoordinateTransform::llaToWorld(lla, origin.value(), model_metric);
            x = world.x();
            y = world.y();
        }
        points.push_back({x, y});
    }
    return points;
}

} // namespace map
} // namespace coastmotionplanning
