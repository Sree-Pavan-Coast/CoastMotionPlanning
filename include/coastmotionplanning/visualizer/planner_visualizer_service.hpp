#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "coastmotionplanning/common/types.hpp"
#include "coastmotionplanning/config/robots_parser.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/planning/behavior_tree_planner_orchestrator.hpp"
#include "coastmotionplanning/planning/hybrid_a_star_planner.hpp"
#include "coastmotionplanning/planning/planner_behavior_set.hpp"
#include "coastmotionplanning/robot/car.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace visualizer {

struct Bounds2d {
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};
};

struct PointDto {
    double x{0.0};
    double y{0.0};
};

struct PoseDto {
    double x{0.0};
    double y{0.0};
    double heading_deg{0.0};
};

struct LaneDto {
    std::vector<PoseDto> poses;
};

struct ZoneDto {
    std::string name;
    std::string type;
    std::string planner_behavior;
    bool reverse_allowed{false};
    std::vector<PointDto> polygon;
    std::vector<LaneDto> lanes;
};

struct VehicleFootprintDto {
    std::string name;
    double width_m{0.0};
    double wheelbase_m{0.0};
    double front_overhang_m{0.0};
    double rear_overhang_m{0.0};
    double total_length_m{0.0};
    double min_turning_radius_m{0.0};
    double max_steer_angle_rad{0.0};
};

struct RobotOptionDto {
    std::string name;
    std::string type;
    bool planning_supported{false};
    VehicleFootprintDto vehicle;
};

struct RobotCatalogResponse {
    std::string default_robot_name;
    std::vector<RobotOptionDto> robots;
};

struct MapLoadRequest {
    std::string filename;
    std::string yaml;
    std::string robot_name;
};

struct MapLoadResponse {
    std::string map_id;
    std::string filename;
    std::string map_name;
    Bounds2d bounds;
    VehicleFootprintDto vehicle;
    std::vector<ZoneDto> zones;
    std::vector<PointDto> search_boundary;  // Concave hull polygon of all zones
};

struct PlanRequest {
    std::string map_id;
    std::string robot_name;
    PoseDto start;
    PoseDto goal;
};

struct PlanResponse {
    bool success{false};
    std::string detail;
    std::string selected_profile;
    std::vector<std::string> attempted_profiles;
    std::vector<PoseDto> path;
    std::vector<std::string> behavior_sequence;
    std::vector<std::string> segment_directions;
};

struct PlannerVisualizerServiceConfig {
    std::filesystem::path configs_root;
    std::string default_car_name{"Pro_XD"};
};

class PlannerVisualizerService {
public:
    explicit PlannerVisualizerService(PlannerVisualizerServiceConfig config);

    RobotCatalogResponse listRobots() const;
    MapLoadResponse loadMap(const MapLoadRequest& request);
    PlanResponse plan(const PlanRequest& request);

    static math::Pose2d makePose(const PoseDto& pose_input, const std::string& label);
    static Bounds2d computeBounds(const std::vector<std::shared_ptr<zones::Zone>>& zones);

private:
    struct LoadedMapState {
        MapLoadResponse response;
        std::vector<std::shared_ptr<zones::Zone>> zones;
    };

    static std::string extractMapName(const std::string& yaml_content,
                                      const std::string& filename);
    static VehicleFootprintDto buildVehicleFootprintDto(
        const config::CarDefinition& car_definition);
    ZoneDto buildZoneDto(const std::shared_ptr<zones::Zone>& zone);
    static PoseDto buildPoseDto(const math::Pose2d& pose);
    static std::string motionDirectionToString(common::MotionDirection direction);
    static std::string zoneTypeName(const std::shared_ptr<zones::Zone>& zone);
    const config::CarDefinition& requireCarDefinition(const std::string& robot_name) const;
    std::string enrichFailureDetail(const std::string& detail,
                                    const math::Pose2d& start_pose,
                                    const math::Pose2d& goal_pose,
                                    const config::CarDefinition& car_definition,
                                    const planning::PlannerBehaviorSet& behavior_set,
                                    const std::shared_ptr<zones::Zone>& start_zone,
                                    const std::shared_ptr<zones::Zone>& goal_zone) const;

    std::shared_ptr<LoadedMapState> requireMap(const std::string& map_id);
    std::string nextMapId();

    PlannerVisualizerServiceConfig config_;
    std::unordered_map<std::string, config::CarDefinition> car_definitions_;
    std::vector<RobotOptionDto> robots_;
    planning::PlannerBehaviorSet base_behavior_set_;
    std::filesystem::path behavior_tree_xml_path_;
    std::mutex maps_mutex_;
    std::unordered_map<std::string, std::shared_ptr<LoadedMapState>> maps_;
    std::atomic<uint64_t> next_map_id_{1};
};

} // namespace visualizer
} // namespace coastmotionplanning
