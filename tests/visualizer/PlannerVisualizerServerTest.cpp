#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <httplib.h>
#include <nlohmann/json.hpp>

#include "coastmotionplanning/visualizer/planner_visualizer_server.hpp"
#include "coastmotionplanning/visualizer/planner_visualizer_service.hpp"

namespace {

using json = nlohmann::json;

using coastmotionplanning::visualizer::PlannerVisualizerServer;
using coastmotionplanning::visualizer::PlannerVisualizerServerConfig;
using coastmotionplanning::visualizer::PlannerVisualizerService;
using coastmotionplanning::visualizer::PlannerVisualizerServiceConfig;

std::string configsRoot() {
    return std::string(COAST_REPO_ROOT) + "/configs";
}

std::string simpleMapYaml() {
    return R"(
maps:
  name: "Server Test Map"
zones:
  - name: "track_zone"
    type: "TrackMainRoad"
    planner_behavior: "primary_profile"
    coordinate_type: "world"
    polygon:
      - [0.0, -4.0]
      - [12.0, -4.0]
      - [12.0, 4.0]
      - [0.0, 4.0]
    lanes:
      - lane_waypoints:
          - [1.0, -1.5]
          - [11.0, -1.5]
      - lane_waypoints:
          - [11.0, 1.5]
          - [1.0, 1.5]
)";
}

TEST(PlannerVisualizerServerTest, MapLoadAndPlanEndpointsWorkEndToEnd) {
    auto service = std::make_shared<PlannerVisualizerService>(PlannerVisualizerServiceConfig{
        configsRoot(),
        "Pro_XD"
    });
    PlannerVisualizerServer server(
        PlannerVisualizerServerConfig{"127.0.0.1", 0, {}},
        service);

    const int port = server.start();
    httplib::Client client("127.0.0.1", port);

    const auto robots_response = client.Get("/api/robots");
    ASSERT_TRUE(robots_response);
    ASSERT_EQ(robots_response->status, 200);

    const auto robots_body = json::parse(robots_response->body);
    EXPECT_EQ(robots_body.at("default_robot_name").get<std::string>(), "Pro_XD");
    ASSERT_FALSE(robots_body.at("robots").empty());

    const auto load_response = client.Post(
        "/api/map/load",
        json{
            {"filename", "server_test.yaml"},
            {"yaml", simpleMapYaml()},
            {"robot_name", "E_Transit"}
        }.dump(),
        "application/json");

    ASSERT_TRUE(load_response);
    ASSERT_EQ(load_response->status, 200);

    const auto load_body = json::parse(load_response->body);
    ASSERT_TRUE(load_body.contains("map_id"));
    ASSERT_TRUE(load_body.contains("vehicle"));
    EXPECT_EQ(load_body.at("vehicle").at("name").get<std::string>(), "E_Transit");
    EXPECT_GT(load_body.at("vehicle").at("width_m").get<double>(), 0.0);
    EXPECT_GT(load_body.at("vehicle").at("max_steer_angle_rad").get<double>(), 0.0);

    const auto plan_response = client.Post(
        "/api/plan",
        json{
            {"map_id", load_body.at("map_id").get<std::string>()},
            {"robot_name", "E_Transit"},
            {"start", {{"x", 2.0}, {"y", 0.0}, {"heading_deg", 0.0}}},
            {"goal", {{"x", 7.0}, {"y", 0.0}, {"heading_deg", 0.0}}}
        }.dump(),
        "application/json");

    ASSERT_TRUE(plan_response);
    ASSERT_EQ(plan_response->status, 200);

    const auto plan_body = json::parse(plan_response->body);
    EXPECT_TRUE(plan_body.at("success").get<bool>()) << plan_body.at("detail").get<std::string>();
    EXPECT_GT(plan_body.at("path").size(), 1u);
}

TEST(PlannerVisualizerServerTest, InvalidRequestsReturnHttp400) {
    auto service = std::make_shared<PlannerVisualizerService>(PlannerVisualizerServiceConfig{
        configsRoot(),
        "Pro_XD"
    });
    PlannerVisualizerServer server(
        PlannerVisualizerServerConfig{"127.0.0.1", 0, {}},
        service);

    const int port = server.start();
    httplib::Client client("127.0.0.1", port);

    const auto bad_map_response = client.Post(
        "/api/map/load",
        json{{"filename", "broken.yaml"}, {"yaml", "zones: ["}}.dump(),
        "application/json");

    ASSERT_TRUE(bad_map_response);
    EXPECT_EQ(bad_map_response->status, 400);

    const auto missing_map_id_response = client.Post(
        "/api/plan",
        json{
            {"start", {{"x", 0.0}, {"y", 0.0}, {"heading_deg", 0.0}}},
            {"goal", {{"x", 1.0}, {"y", 0.0}, {"heading_deg", 0.0}}}
        }.dump(),
        "application/json");

    ASSERT_TRUE(missing_map_id_response);
    EXPECT_EQ(missing_map_id_response->status, 400);
}

} // namespace
