#include "coastmotionplanning/visualizer/planner_visualizer_service.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "coastmotionplanning/config/robots_parser.hpp"
#include "coastmotionplanning/costs/zone_selector.hpp"
#include "coastmotionplanning/map/map_parser.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

namespace coastmotionplanning {
namespace visualizer {

namespace {

struct RobotCatalogData {
    std::unordered_map<std::string, config::CarDefinition> car_definitions;
    std::vector<RobotOptionDto> robots;
};

std::filesystem::path requireConfigPath(const std::filesystem::path& configs_root,
                                        const std::string& filename) {
    if (configs_root.empty()) {
        throw std::runtime_error("PlannerVisualizerService requires a configs_root path.");
    }
    const auto path = configs_root / filename;
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("Missing visualizer config file: " + path.string());
    }
    return path;
}

void extendBounds(Bounds2d& bounds,
                  bool& has_points,
                  double x,
                  double y) {
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return;
    }

    if (!has_points) {
        bounds.min_x = x;
        bounds.max_x = x;
        bounds.min_y = y;
        bounds.max_y = y;
        has_points = true;
        return;
    }

    bounds.min_x = std::min(bounds.min_x, x);
    bounds.max_x = std::max(bounds.max_x, x);
    bounds.min_y = std::min(bounds.min_y, y);
    bounds.max_y = std::max(bounds.max_y, y);
}

VehicleFootprintDto makeVehicleFootprintDto(const config::CarDefinition& car_definition) {
    return VehicleFootprintDto{
        car_definition.name,
        car_definition.width_m,
        car_definition.wheelbase_m,
        car_definition.front_overhang_m,
        car_definition.rear_overhang_m,
        car_definition.wheelbase_m +
            car_definition.front_overhang_m +
            car_definition.rear_overhang_m,
        car_definition.minTurningRadiusMeters(),
        car_definition.maxSteerAngleRadians()
    };
}

RobotCatalogData loadRobotCatalog(const PlannerVisualizerServiceConfig& config) {
    const auto robots_path = requireConfigPath(config.configs_root, "robots.yaml");
    const auto robot_definitions = config::RobotsParser::parse(robots_path.string());
    const auto car_definitions = config::RobotsParser::parseCarDefinitions(robots_path.string());

    RobotCatalogData catalog;
    catalog.robots.reserve(robot_definitions.size());
    for (const auto& car_definition : car_definitions) {
        catalog.car_definitions.emplace(car_definition.name, car_definition);
    }

    for (const auto& robot_definition : robot_definitions) {
        RobotOptionDto option;
        option.name = robot_definition.name;
        option.type = robot_definition.type;

        const auto car_it = catalog.car_definitions.find(robot_definition.name);
        if (car_it != catalog.car_definitions.end()) {
            option.planning_supported = true;
            option.vehicle = makeVehicleFootprintDto(car_it->second);
        } else {
            option.vehicle.name = robot_definition.name;
        }

        catalog.robots.push_back(std::move(option));
    }

    return catalog;
}

planning::PlannerBehaviorSet loadBaseBehaviorSet(const PlannerVisualizerServiceConfig& config) {
    return planning::PlannerBehaviorSet::loadFromFile(
        requireConfigPath(config.configs_root, "planner_behaviors.yaml").string());
}

planning::PlannerBehaviorSet buildBehaviorSetForCar(
    planning::PlannerBehaviorSet behavior_set,
    const config::CarDefinition& car_definition) {
    behavior_set.overrideMotionPrimitiveConstraints(
        car_definition.minTurningRadiusMeters(),
        car_definition.maxSteerAngleRadians());
    behavior_set.setMinimumPlanningTimeMs(1500);
    return behavior_set;
}

double normalizedHeadingDeltaDegrees(const math::Pose2d& start_pose,
                                     const math::Pose2d& goal_pose) {
    double delta = goal_pose.theta.degrees() - start_pose.theta.degrees();
    while (delta <= -180.0) {
        delta += 360.0;
    }
    while (delta > 180.0) {
        delta -= 360.0;
    }
    return std::abs(delta);
}

} // namespace

PlannerVisualizerService::PlannerVisualizerService(PlannerVisualizerServiceConfig config)
    : config_(std::move(config)),
      base_behavior_set_(loadBaseBehaviorSet(config_)),
      behavior_tree_xml_path_(
          requireConfigPath(config_.configs_root, "behavior_trees/planner_behavior_tree.xml")) {
    auto catalog = loadRobotCatalog(config_);
    car_definitions_ = std::move(catalog.car_definitions);
    robots_ = std::move(catalog.robots);

    // Fail fast if the configured default robot is missing or unsupported.
    (void)requireCarDefinition(config_.default_car_name);
}

RobotCatalogResponse PlannerVisualizerService::listRobots() const {
    return RobotCatalogResponse{
        config_.default_car_name,
        robots_
    };
}

MapLoadResponse PlannerVisualizerService::loadMap(const MapLoadRequest& request) {
    if (request.yaml.empty()) {
        throw std::runtime_error("Map YAML content is empty.");
    }

    const std::string selected_robot_name =
        request.robot_name.empty() ? config_.default_car_name : request.robot_name;
    const auto& car_definition = requireCarDefinition(selected_robot_name);

    const auto source_label = request.filename.empty() ? std::string("<uploaded map>")
                                                       : request.filename;
    const auto zones = map::MapParser::parseFromString(request.yaml, source_label);
    if (zones.empty()) {
        throw std::runtime_error("Map does not define any zones.");
    }

    auto loaded_map = std::make_shared<LoadedMapState>();
    loaded_map->zones = zones;
    loaded_map->response.map_id = nextMapId();
    loaded_map->response.filename = request.filename;
    loaded_map->response.map_name = extractMapName(request.yaml, request.filename);
    loaded_map->response.bounds = computeBounds(zones);
    loaded_map->response.vehicle = buildVehicleFootprintDto(car_definition);

    loaded_map->response.zones.reserve(zones.size());
    for (const auto& zone : zones) {
        loaded_map->response.zones.push_back(buildZoneDto(zone));
    }

    // Compute the concave hull (search space boundary) over all zone polygons
    {
        std::vector<geometry::Polygon2d> zone_polygons;
        zone_polygons.reserve(zones.size());
        for (const auto& zone : zones) {
            zone_polygons.push_back(zone->getPolygon());
        }
        const auto hull = costs::ZoneSelector::computeConcaveHull(zone_polygons, 0.0);
        const auto& ring = hull.outer();
        loaded_map->response.search_boundary.reserve(ring.size());
        for (size_t i = 0; i < ring.size(); ++i) {
            // Skip the Boost.Geometry closing vertex (duplicate of first point)
            if (i == ring.size() - 1 && ring.size() > 1 &&
                std::abs(ring[i].x() - ring[0].x()) < 1e-9 &&
                std::abs(ring[i].y() - ring[0].y()) < 1e-9) {
                continue;
            }
            loaded_map->response.search_boundary.push_back(
                PointDto{ring[i].x(), ring[i].y()});
        }
    }

    std::lock_guard<std::mutex> lock(maps_mutex_);
    maps_[loaded_map->response.map_id] = loaded_map;
    return loaded_map->response;
}

PlanResponse PlannerVisualizerService::plan(const PlanRequest& request) {
    if (request.map_id.empty()) {
        throw std::runtime_error("Plan request is missing map_id.");
    }

    const auto loaded_map = requireMap(request.map_id);
    const std::string selected_robot_name =
        request.robot_name.empty() ? loaded_map->response.vehicle.name : request.robot_name;
    const auto& car_definition = requireCarDefinition(selected_robot_name);
    const auto car = car_definition.buildCar();
    const auto behavior_set = buildBehaviorSetForCar(base_behavior_set_, car_definition);
    const planning::BehaviorTreePlannerOrchestrator orchestrator(
        behavior_tree_xml_path_.string(),
        behavior_set);
    const auto start_pose = makePose(request.start, "start");
    const auto goal_pose = makePose(request.goal, "goal");

    const auto start_zone = costs::ZoneSelector::findContainingZone(
        geometry::Point2d(start_pose.x, start_pose.y), loaded_map->zones);
    if (start_zone == nullptr) {
        throw std::runtime_error("Start pose is not inside any map zone.");
    }

    const auto goal_zone = costs::ZoneSelector::findContainingZone(
        geometry::Point2d(goal_pose.x, goal_pose.y), loaded_map->zones);
    if (goal_zone == nullptr) {
        throw std::runtime_error("Goal pose is not inside any map zone.");
    }

    planning::HybridAStarPlannerResult last_planner_result;
    auto runner = [&](const planning::PlanningAttempt& attempt) {
        planning::HybridAStarPlanner planner(car, loaded_map->zones, behavior_set);
        planning::HybridAStarPlannerRequest planner_request;
        planner_request.start = start_pose;
        planner_request.goal = goal_pose;
        planner_request.initial_behavior_name = attempt.profile;

        last_planner_result = planner.plan(planner_request);
        return planning::PlannerRunResult{
            last_planner_result.success,
            last_planner_result.detail
        };
    };

    planning::PlanningRequestContext request_context;
    request_context.intent = planning::PlanningIntent::NORMAL;
    request_context.start_zone = start_zone;
    request_context.goal_zone = goal_zone;

    const auto orchestration_result = orchestrator.run(request_context, runner);

    PlanResponse response;
    response.success = orchestration_result.success;
    response.detail = orchestration_result.success
        ? orchestration_result.detail
        : enrichFailureDetail(
            orchestration_result.detail,
            start_pose,
            goal_pose,
            car_definition,
            behavior_set,
            start_zone,
            goal_zone);
    response.selected_profile = orchestration_result.selected_profile;
    response.attempted_profiles = orchestration_result.attempted_profiles;

    if (orchestration_result.success && last_planner_result.success) {
        response.path.reserve(last_planner_result.poses.size());
        for (const auto& pose : last_planner_result.poses) {
            response.path.push_back(buildPoseDto(pose));
        }
        response.behavior_sequence = last_planner_result.behavior_sequence;
        response.segment_directions.reserve(last_planner_result.segment_directions.size());
        for (const auto direction : last_planner_result.segment_directions) {
            response.segment_directions.push_back(motionDirectionToString(direction));
        }
    }

    return response;
}

math::Pose2d PlannerVisualizerService::makePose(const PoseDto& pose_input,
                                                const std::string& label) {
    if (!std::isfinite(pose_input.x) || !std::isfinite(pose_input.y) ||
        !std::isfinite(pose_input.heading_deg)) {
        throw std::runtime_error("Pose '" + label + "' contains non-finite values.");
    }

    return math::Pose2d{
        pose_input.x,
        pose_input.y,
        math::Angle::from_degrees(pose_input.heading_deg).normalized()
    };
}

Bounds2d PlannerVisualizerService::computeBounds(
    const std::vector<std::shared_ptr<zones::Zone>>& zones) {
    Bounds2d bounds;
    bool has_points = false;

    for (const auto& zone : zones) {
        for (const auto& point : zone->getPolygon().outer()) {
            extendBounds(bounds, has_points, point.x(), point.y());
        }

        const auto track = std::dynamic_pointer_cast<zones::TrackMainRoad>(zone);
        if (track == nullptr) {
            continue;
        }

        for (const auto& lane : track->getLanes()) {
            for (const auto& pose : lane) {
                extendBounds(bounds, has_points, pose.x, pose.y);
            }
        }
    }

    if (!has_points) {
        throw std::runtime_error("Map does not contain any drawable geometry.");
    }

    return bounds;
}

std::string PlannerVisualizerService::extractMapName(const std::string& yaml_content,
                                                     const std::string& filename) {
    try {
        const auto config = YAML::Load(yaml_content);
        const auto maps_node = config["maps"];
        if (maps_node && maps_node["name"] && maps_node["name"].IsScalar()) {
            const auto name = maps_node["name"].as<std::string>("");
            if (!name.empty()) {
                return name;
            }
        }
    } catch (const std::exception&) {
        // Fall back to filename or placeholder if the caller only needs a label.
    }

    if (!filename.empty()) {
        return std::filesystem::path(filename).stem().string();
    }
    return "Untitled Map";
}

VehicleFootprintDto PlannerVisualizerService::buildVehicleFootprintDto(
    const config::CarDefinition& car_definition) {
    return makeVehicleFootprintDto(car_definition);
}

ZoneDto PlannerVisualizerService::buildZoneDto(const std::shared_ptr<zones::Zone>& zone) {
    ZoneDto dto;
    dto.name = zone->getName().value_or(zoneTypeName(zone));
    dto.type = zoneTypeName(zone);
    dto.planner_behavior = zone->getResolvedPlannerBehavior();
    dto.reverse_allowed = zone->isReverseAllowed();
    if (base_behavior_set_.contains(dto.planner_behavior)) {
        dto.reverse_allowed =
            dto.reverse_allowed &&
            !base_behavior_set_.get(dto.planner_behavior).planner.only_forward_path;
    }

    const auto& polygon = zone->getPolygon().outer();
    dto.polygon.reserve(polygon.size());
    for (size_t i = 0; i < polygon.size(); ++i) {
        // Skip the Boost.Geometry closing vertex (duplicate of first point)
        if (i == polygon.size() - 1 && polygon.size() > 1 &&
            std::abs(polygon[i].x() - polygon[0].x()) < 1e-9 &&
            std::abs(polygon[i].y() - polygon[0].y()) < 1e-9) {
            continue;
        }
        dto.polygon.push_back(PointDto{polygon[i].x(), polygon[i].y()});
    }

    const auto track = std::dynamic_pointer_cast<zones::TrackMainRoad>(zone);
    if (track != nullptr) {
        dto.lanes.reserve(track->getLanes().size());
        for (const auto& lane : track->getLanes()) {
            LaneDto lane_dto;
            lane_dto.poses.reserve(lane.size());
            for (const auto& pose : lane) {
                lane_dto.poses.push_back(buildPoseDto(pose));
            }
            dto.lanes.push_back(std::move(lane_dto));
        }
    }

    return dto;
}

PoseDto PlannerVisualizerService::buildPoseDto(const math::Pose2d& pose) {
    return PoseDto{
        pose.x,
        pose.y,
        pose.theta.normalized().degrees()
    };
}

std::string PlannerVisualizerService::motionDirectionToString(common::MotionDirection direction) {
    switch (direction) {
    case common::MotionDirection::Forward:
        return "Forward";
    case common::MotionDirection::Reverse:
        return "Reverse";
    }
    return "Forward";
}

std::string PlannerVisualizerService::zoneTypeName(
    const std::shared_ptr<zones::Zone>& zone) {
    if (std::dynamic_pointer_cast<zones::TrackMainRoad>(zone) != nullptr) {
        return "TrackMainRoad";
    }
    if (std::dynamic_pointer_cast<zones::ManeuveringZone>(zone) != nullptr) {
        return "ManeuveringZone";
    }
    return "Zone";
}

std::string PlannerVisualizerService::enrichFailureDetail(
    const std::string& detail,
    const math::Pose2d& start_pose,
    const math::Pose2d& goal_pose,
    const config::CarDefinition& car_definition,
    const planning::PlannerBehaviorSet& behavior_set,
    const std::shared_ptr<zones::Zone>& start_zone,
    const std::shared_ptr<zones::Zone>& goal_zone) const {
    std::string message = detail;

    if (detail.find("collision") != std::string::npos) {
        message += " Rear-axle poses are not footprint centers: " + car_definition.name +
                   " extends " + std::to_string(car_definition.rear_overhang_m) +
                   " m behind the pose and " +
                   std::to_string(car_definition.wheelbase_m +
                                  car_definition.front_overhang_m) +
                   " m forward, so clicks near a zone boundary can still collide.";
        return message;
    }

    const double heading_delta_deg = normalizedHeadingDeltaDegrees(start_pose, goal_pose);
    const double turning_radius_m = car_definition.minTurningRadiusMeters();
    bool goal_profile_is_forward_only = false;
    if (goal_zone != nullptr) {
        const std::string goal_behavior = goal_zone->getResolvedPlannerBehavior();
        if (behavior_set.contains(goal_behavior)) {
            goal_profile_is_forward_only =
                behavior_set.get(goal_behavior).planner.only_forward_path;
        }
    }
    if (detail.find("timed out") != std::string::npos ||
        detail.find("exhausted the search space") != std::string::npos) {
        message += " Visualizer planning currently uses the selected car's turning radius (" +
                   std::to_string(turning_radius_m) + " m for " +
                   car_definition.name + ").";
        if (start_zone != nullptr && goal_zone != nullptr &&
            zoneTypeName(start_zone) == "ManeuveringZone" &&
            zoneTypeName(goal_zone) == "ManeuveringZone" &&
            heading_delta_deg >= 120.0) {
            message += " This request changes heading by " +
                       std::to_string(heading_delta_deg) +
                       " deg inside a maneuvering zone, which may simply not have enough room.";
        } else if (goal_profile_is_forward_only) {
            message +=
                " Reverse motion is disabled for the goal context, so large heading changes can be infeasible.";
        }
    }

    return message;
}

const config::CarDefinition& PlannerVisualizerService::requireCarDefinition(
    const std::string& robot_name) const {
    const auto car_it = car_definitions_.find(robot_name);
    if (car_it != car_definitions_.end()) {
        return car_it->second;
    }

    for (const auto& robot : robots_) {
        if (robot.name == robot_name) {
            throw std::runtime_error(
                "Robot '" + robot_name +
                "' is defined in robots.yaml but the visualizer currently supports only Car robots.");
        }
    }

    throw std::runtime_error("Unknown robot '" + robot_name + "'.");
}

std::shared_ptr<PlannerVisualizerService::LoadedMapState> PlannerVisualizerService::requireMap(
    const std::string& map_id) {
    std::lock_guard<std::mutex> lock(maps_mutex_);
    const auto it = maps_.find(map_id);
    if (it == maps_.end()) {
        throw std::runtime_error("Unknown map_id '" + map_id + "'.");
    }
    return it->second;
}

std::string PlannerVisualizerService::nextMapId() {
    return "map-" + std::to_string(next_map_id_.fetch_add(1));
}

} // namespace visualizer
} // namespace coastmotionplanning
