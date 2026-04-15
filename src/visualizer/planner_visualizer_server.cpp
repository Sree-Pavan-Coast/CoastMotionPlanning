#include "coastmotionplanning/visualizer/planner_visualizer_server.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "coastmotionplanning/visualizer/planner_visualizer_service.hpp"

namespace coastmotionplanning {
namespace visualizer {

namespace {

using json = nlohmann::json;

json makeErrorJson(const std::string& detail) {
    return json{{"success", false}, {"detail", detail}};
}

json poseToJson(const PoseDto& pose) {
    return json{
        {"x", pose.x},
        {"y", pose.y},
        {"heading_deg", pose.heading_deg}
    };
}

json laneToJson(const LaneDto& lane) {
    json poses = json::array();
    for (const auto& pose : lane.poses) {
        poses.push_back(poseToJson(pose));
    }
    return json{{"poses", poses}};
}

json vehicleToJson(const VehicleFootprintDto& vehicle) {
    return json{
        {"name", vehicle.name},
        {"width_m", vehicle.width_m},
        {"wheelbase_m", vehicle.wheelbase_m},
        {"front_overhang_m", vehicle.front_overhang_m},
        {"rear_overhang_m", vehicle.rear_overhang_m},
        {"total_length_m", vehicle.total_length_m},
        {"min_turning_radius_m", vehicle.min_turning_radius_m},
        {"max_steer_angle_rad", vehicle.max_steer_angle_rad}
    };
}

json robotToJson(const RobotOptionDto& robot) {
    return json{
        {"name", robot.name},
        {"type", robot.type},
        {"planning_supported", robot.planning_supported},
        {"vehicle", vehicleToJson(robot.vehicle)}
    };
}

json robotCatalogToJson(const RobotCatalogResponse& response) {
    json robots = json::array();
    for (const auto& robot : response.robots) {
        robots.push_back(robotToJson(robot));
    }

    return json{
        {"default_robot_name", response.default_robot_name},
        {"robots", robots}
    };
}

json zoneToJson(const ZoneDto& zone) {
    json polygon = json::array();
    for (const auto& point : zone.polygon) {
        polygon.push_back(json{{"x", point.x}, {"y", point.y}});
    }

    json lanes = json::array();
    for (const auto& lane : zone.lanes) {
        lanes.push_back(laneToJson(lane));
    }

    return json{
        {"name", zone.name},
        {"type", zone.type},
        {"planner_behavior", zone.planner_behavior},
        {"reverse_allowed", zone.reverse_allowed},
        {"polygon", polygon},
        {"lanes", lanes}
    };
}

json mapLoadResponseToJson(const MapLoadResponse& response) {
    json zones = json::array();
    for (const auto& zone : response.zones) {
        zones.push_back(zoneToJson(zone));
    }

    json search_boundary = json::array();
    for (const auto& point : response.search_boundary) {
        search_boundary.push_back(json{{"x", point.x}, {"y", point.y}});
    }

    return json{
        {"map_id", response.map_id},
        {"filename", response.filename},
        {"map_name", response.map_name},
        {"bounds", {
            {"min_x", response.bounds.min_x},
            {"min_y", response.bounds.min_y},
            {"max_x", response.bounds.max_x},
            {"max_y", response.bounds.max_y}
        }},
        {"vehicle", vehicleToJson(response.vehicle)},
        {"zones", zones},
        {"search_boundary", search_boundary}
    };
}

json planResponseToJson(const PlanResponse& response) {
    json path = json::array();
    for (const auto& pose : response.path) {
        path.push_back(poseToJson(pose));
    }

    return json{
        {"success", response.success},
        {"detail", response.detail},
        {"selected_profile", response.selected_profile},
        {"attempted_profiles", response.attempted_profiles},
        {"path", path},
        {"behavior_sequence", response.behavior_sequence},
        {"segment_directions", response.segment_directions},
        {"debug_mode", response.debug_mode},
        {"debug_report_path", response.debug_report_path}
    };
}

PoseDto poseFromJson(const json& root, const std::string& field_name) {
    if (!root.contains(field_name) || !root.at(field_name).is_object()) {
        throw std::runtime_error("Request body is missing object field '" + field_name + "'.");
    }

    const auto& pose = root.at(field_name);
    for (const auto key : {"x", "y", "heading_deg"}) {
        if (!pose.contains(key) || !pose.at(key).is_number()) {
            throw std::runtime_error(
                "Pose '" + field_name + "' is missing numeric field '" + std::string(key) + "'.");
        }
    }

    return PoseDto{
        pose.at("x").get<double>(),
        pose.at("y").get<double>(),
        pose.at("heading_deg").get<double>()
    };
}

} // namespace

PlannerVisualizerServer::PlannerVisualizerServer(
    PlannerVisualizerServerConfig config,
    std::shared_ptr<PlannerVisualizerService> service)
    : config_(std::move(config)),
      service_(std::move(service)),
      server_(std::make_unique<httplib::Server>()) {
    if (service_ == nullptr) {
        throw std::runtime_error("PlannerVisualizerServer requires a service instance.");
    }

    if (!config_.asset_dir.empty()) {
        if (!std::filesystem::exists(config_.asset_dir)) {
            throw std::runtime_error(
                "Planner visualizer asset directory does not exist: " + config_.asset_dir.string());
        }
        if (!server_->set_mount_point("/", config_.asset_dir.string())) {
            throw std::runtime_error(
                "Failed to mount planner visualizer assets from: " + config_.asset_dir.string());
        }
    }

    registerRoutes();
}

PlannerVisualizerServer::~PlannerVisualizerServer() {
    stop();
}

int PlannerVisualizerServer::start() {
    if (bound_port_ != 0) {
        return bound_port_;
    }

    if (config_.port == 0) {
        bound_port_ = server_->bind_to_any_port(config_.host);
        if (bound_port_ <= 0) {
            throw std::runtime_error("Failed to bind planner visualizer server to an ephemeral port.");
        }
    } else {
        if (!server_->bind_to_port(config_.host, config_.port)) {
            throw std::runtime_error(
                "Failed to bind planner visualizer server to " + config_.host + ":" +
                std::to_string(config_.port));
        }
        bound_port_ = config_.port;
    }

    server_thread_ = std::thread([this]() {
        server_->listen_after_bind();
    });

    httplib::Client client(config_.host, bound_port_);
    client.set_connection_timeout(std::chrono::milliseconds(200));

    for (int attempt = 0; attempt < 40; ++attempt) {
        const auto response = client.Get("/api/health");
        if (response && response->status == 200) {
            return bound_port_;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    stop();
    throw std::runtime_error("Planner visualizer server failed to become ready.");
}

void PlannerVisualizerServer::stop() {
    if (server_ != nullptr) {
        server_->stop();
    }

    if (server_thread_.joinable()) {
        server_thread_.join();
    }

    bound_port_ = 0;
}

void PlannerVisualizerServer::wait() {
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
}

std::string PlannerVisualizerServer::baseUrl() const {
    if (bound_port_ == 0) {
        return {};
    }
    return "http://" + config_.host + ":" + std::to_string(bound_port_);
}

void PlannerVisualizerServer::registerRoutes() {
    server_->Get("/api/health", [](const httplib::Request&, httplib::Response& response) {
        response.set_content(R"({"status":"ok"})", "application/json");
    });

    server_->Get("/api/robots", [this](const httplib::Request&,
                                       httplib::Response& response) {
        try {
            response.set_content(
                robotCatalogToJson(service_->listRobots()).dump(),
                "application/json");
        } catch (const std::exception& e) {
            response.status = 400;
            response.set_content(makeErrorJson(e.what()).dump(), "application/json");
        }
    });

    server_->Post("/api/map/load", [this](const httplib::Request& request,
                                          httplib::Response& response) {
        try {
            const auto body = json::parse(request.body);
            const auto result = service_->loadMap(MapLoadRequest{
                body.value("filename", std::string{}),
                body.value("yaml", std::string{}),
                body.value("robot_name", std::string{})
            });
            response.set_content(mapLoadResponseToJson(result).dump(), "application/json");
        } catch (const std::exception& e) {
            response.status = 400;
            response.set_content(makeErrorJson(e.what()).dump(), "application/json");
        }
    });

    server_->Post("/api/plan", [this](const httplib::Request& request,
                                      httplib::Response& response) {
        try {
            const auto body = json::parse(request.body);
            const auto result = service_->plan(PlanRequest{
                body.value("map_id", std::string{}),
                body.value("robot_name", std::string{}),
                poseFromJson(body, "start"),
                poseFromJson(body, "goal")
            });
            response.set_content(planResponseToJson(result).dump(), "application/json");
        } catch (const std::exception& e) {
            response.status = 400;
            response.set_content(makeErrorJson(e.what()).dump(), "application/json");
        }
    });
}

} // namespace visualizer
} // namespace coastmotionplanning
