#include "coastmotionplanning/visualizer/planner_visualizer_service.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "coastmotionplanning/config/robots_parser.hpp"
#include "coastmotionplanning/costs/zone_selector.hpp"
#include "coastmotionplanning/map/map_parser.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

namespace coastmotionplanning {
namespace visualizer {

namespace {

using json = nlohmann::json;

struct RobotCatalogData {
    std::unordered_map<std::string, config::CarDefinition> car_definitions;
    std::vector<RobotOptionDto> robots;
};

json poseToJson(const math::Pose2d& pose) {
    return json{
        {"x", pose.x},
        {"y", pose.y},
        {"heading_deg", pose.theta.normalized().degrees()}
    };
}

json profilingScopesToJson(
    const std::vector<common::ProfilingScopeSummary>& profiling_scopes) {
    json scopes = json::array();
    for (const auto& scope : profiling_scopes) {
        scopes.push_back(json{
            {"scope_name", scope.scope_name},
            {"count", scope.count},
            {"total_ms", scope.total_ms},
            {"avg_ms", scope.avg_ms},
            {"max_ms", scope.max_ms}
        });
    }
    return scopes;
}

json profileToJson(const planning::PlannerBehaviorProfile& profile) {
    return json{
        {"planner", {
            {"max_planning_time_ms", profile.planner.max_planning_time_ms},
            {"xy_grid_resolution_m", profile.planner.xy_grid_resolution_m},
            {"yaw_grid_resolution_deg", profile.planner.yaw_grid_resolution_deg},
            {"step_size_m", profile.planner.step_size_m},
            {"only_forward_path", profile.planner.only_forward_path},
            {"weight_forward", profile.planner.weight_forward},
            {"weight_reverse", profile.planner.weight_reverse},
            {"weight_steer", profile.planner.weight_steer},
            {"weight_steer_change", profile.planner.weight_steer_change},
            {"weight_gear_change", profile.planner.weight_gear_change},
            {"analytic_expansion_max_length_m", profile.planner.analytic_expansion_max_length_m},
            {"analytic_expansion_ratio", profile.planner.analytic_expansion_ratio},
            {"min_path_len_in_same_motion", profile.planner.min_path_len_in_same_motion},
            {"min_goal_straight_approach_m",
             profile.planner.min_goal_straight_approach_m},
            {"analytic_shot", profile.planner.analytic_shot},
            {"near_goal_analytic_expansion", profile.planner.near_goal_analytic_expansion},
            {"near_goal_analytic_radius_m", profile.planner.near_goal_analytic_radius_m},
            {"weight_lane_centerline", profile.planner.weight_lane_centerline},
            {"lane_heading_bias_weight", profile.planner.lane_heading_bias_weight},
            {"max_cross_track_error_m", profile.planner.max_cross_track_error_m},
            {"lane_primitive_suppression", profile.planner.lane_primitive_suppression}
        }},
        {"costmap", {
            {"resolution_m", profile.costmap.resolution_m},
            {"inflation_radius_m", profile.costmap.inflation_radius_m},
            {"inscribed_radius_m", profile.costmap.inscribed_radius_m},
            {"cost_scaling_factor", profile.costmap.cost_scaling_factor},
            {"max_lane_cost", profile.costmap.max_lane_cost},
            {"max_lane_half_width_m", profile.costmap.max_lane_half_width_m}
        }},
        {"collision_checker", {
            {"collision_mode", profile.collision_checker.collision_mode},
            {"lethal_threshold", profile.collision_checker.lethal_threshold},
            {"margin_m", profile.collision_checker.margin_m}
        }},
        {"motion_primitives", {
            {"num_angle_bins", profile.motion_primitives.num_angle_bins},
            {"min_turning_radius_m", profile.motion_primitives.min_turning_radius_m},
            {"max_steer_angle_rad", profile.motion_primitives.max_steer_angle_rad}
        }},
        {"non_holonomic_heuristic", {
            {"lut_grid_size", profile.non_holonomic_heuristic.lut_grid_size},
            {"lut_cell_size_m", profile.non_holonomic_heuristic.lut_cell_size_m},
            {"hitch_angle_penalty_factor", profile.non_holonomic_heuristic.hitch_angle_penalty_factor}
        }},
        {"active_layers", std::vector<std::string>(
            profile.active_layers.begin(), profile.active_layers.end())}
    };
}

json plannerDebugTraceToJson(const planning::HybridAStarPlannerDebugTrace& trace) {
    json frontier_summaries = json::array();
    for (const auto& frontier : trace.frontier_summaries) {
        frontier_summaries.push_back(json{
            {"frontier_id", frontier.frontier_id},
            {"frontier_role", frontier.frontier_role},
            {"zone_name", frontier.zone_name},
            {"behavior_name", frontier.behavior_name},
            {"enqueued", frontier.enqueued},
            {"popped", frontier.popped},
            {"stale_entries_skipped", frontier.stale_entries_skipped},
            {"closed_set_size", frontier.closed_set_size},
            {"open_queue_peak_size", frontier.open_queue_peak_size},
            {"first_enqueue_ms", frontier.first_enqueue_ms},
            {"first_pop_ms", frontier.first_pop_ms},
            {"profiling", {{"scopes", profilingScopesToJson(frontier.profiling_scopes)}}}
        });
    }

    json frontier_handoffs = json::array();
    for (const auto& handoff : trace.frontier_handoffs) {
        frontier_handoffs.push_back(json{
            {"from_frontier_id", handoff.from_frontier_id},
            {"to_frontier_id", handoff.to_frontier_id},
            {"transfer_count", handoff.transfer_count},
            {"first_transfer_ms", handoff.first_transfer_ms}
        });
    }

    json expansions = json::array();
    for (const auto& expansion : trace.expansions) {
        json primitive_events = json::array();
        for (const auto& primitive_event : expansion.primitive_events) {
            primitive_events.push_back(json{
                {"primitive_index", primitive_event.primitive_index},
                {"turn_direction", primitive_event.turn_direction},
                {"successor_pose", poseToJson(primitive_event.successor_pose)},
                {"outcome", primitive_event.outcome},
                {"detail", primitive_event.detail},
                {"resolved_behavior_name", primitive_event.resolved_behavior_name},
                {"resolved_zone_name", primitive_event.resolved_zone_name},
                {"travel_m", primitive_event.travel_m},
                {"edge_cost", primitive_event.edge_cost},
                {"new_g", primitive_event.new_g},
                {"successor_h", primitive_event.successor_h}
            });
        }

        expansions.push_back(json{
            {"expansion_index", expansion.expansion_index},
            {"node_index", expansion.node_index},
            {"frontier_id", expansion.frontier_id},
            {"frontier_role", expansion.frontier_role},
            {"pose", poseToJson(expansion.pose)},
            {"zone_name", expansion.zone_name},
            {"behavior_name", expansion.behavior_name},
            {"g", expansion.g},
            {"h", expansion.h},
            {"f", expansion.f},
            {"distance_to_goal_m", expansion.distance_to_goal_m},
            {"open_queue_size_after_pop", expansion.open_queue_size_after_pop},
            {"goal_satisfied", expansion.goal_satisfied},
            {"terminal_motion_valid", expansion.terminal_motion_valid},
            {"lane_following_candidate", expansion.lane_following_candidate},
            {"lane_suppression_forward_success", expansion.lane_suppression_forward_success},
            {"lane_suppression_fallback", expansion.lane_suppression_fallback},
            {"analytic_attempted", expansion.analytic_attempted},
            {"transition_entry_behavior_active", expansion.transition_entry_behavior_active},
            {"transition_entry_behavior_name", expansion.transition_entry_behavior_name},
            {"transition_steady_behavior_name", expansion.transition_steady_behavior_name},
            {"transition_entry_station_m", expansion.transition_entry_station_m},
            {"transition_track_station_m", expansion.transition_track_station_m},
            {"transition_lane_distance_m", expansion.transition_lane_distance_m},
            {"transition_lane_heading_error_rad", expansion.transition_lane_heading_error_rad},
            {"transition_promotion_reason", expansion.transition_promotion_reason},
            {"analytic_event", {
                {"attempted", expansion.analytic_event.attempted},
                {"trigger", expansion.analytic_event.trigger},
                {"distance_to_goal_m", expansion.analytic_event.distance_to_goal_m},
                {"path_length_m", expansion.analytic_event.path_length_m},
                {"waypoint_count", expansion.analytic_event.waypoint_count},
                {"outcome", expansion.analytic_event.outcome},
                {"detail", expansion.analytic_event.detail}
            }},
            {"primitive_events", primitive_events}
        });
    }

    return json{
        {"initial_behavior_name", trace.initial_behavior_name},
        {"start_zone_name", trace.start_zone_name},
        {"goal_zone_name", trace.goal_zone_name},
        {"selected_zone_names", trace.selected_zone_names},
        {"timing_ms", {
            {"zone_selection", trace.zone_selection_ms},
            {"costmap_build", trace.costmap_build_ms},
            {"heuristic_setup", trace.heuristic_setup_ms},
            {"search_loop", trace.search_loop_ms},
            {"total_planning", trace.total_planning_ms}
        }},
        {"search_summary", {
            {"nodes_allocated", trace.nodes_allocated},
            {"unique_state_count", trace.unique_state_count},
            {"open_queue_peak_size", trace.open_queue_peak_size},
            {"expansions_popped", trace.expansions_popped},
            {"stale_entries_skipped", trace.stale_entries_skipped},
            {"goal_checks", trace.goal_checks},
            {"goal_hits", trace.goal_hits}
        }},
        {"lane_summary", {
            {"lane_following_candidates", trace.lane_following_candidates},
            {"lane_suppression_forward_only_applied", trace.lane_suppression_forward_only_applied},
            {"lane_suppression_fallbacks", trace.lane_suppression_fallbacks}
        }},
        {"analytic_summary", {
            {"attempts", trace.analytic_attempts},
            {"successes", trace.analytic_successes},
            {"fail_no_ompl", trace.analytic_fail_no_ompl},
            {"fail_same_motion_guard", trace.analytic_fail_same_motion_guard},
            {"fail_path_length", trace.analytic_fail_path_length},
            {"fail_out_of_bounds", trace.analytic_fail_out_of_bounds},
            {"fail_collision", trace.analytic_fail_collision},
            {"fail_validation", trace.analytic_fail_validation}
        }},
        {"primitive_summary", {
            {"out_of_bounds", trace.primitive_out_of_bounds},
            {"cross_track_pruned", trace.primitive_cross_track_pruned},
            {"behavior_unresolved", trace.primitive_behavior_unresolved},
            {"primitive_disallowed", trace.primitive_disallowed},
            {"collision", trace.primitive_collision},
            {"motion_change_blocked", trace.primitive_motion_change_blocked},
            {"dominated", trace.primitive_dominated},
            {"enqueued", trace.primitive_enqueued}
        }},
        {"profiling", {
            {"scopes", profilingScopesToJson(trace.profiling_scopes)}
        }},
        {"frontier_summaries", frontier_summaries},
        {"frontier_handoffs", frontier_handoffs},
        {"terminal_reason", trace.terminal_reason},
        {"expansions", expansions}
    };
}

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

std::vector<PointDto> polygonToPointDtos(const geometry::Polygon2d& polygon) {
    std::vector<PointDto> points;
    const auto& ring = polygon.outer();
    points.reserve(ring.size());
    for (size_t i = 0; i < ring.size(); ++i) {
        if (i == ring.size() - 1 && ring.size() > 1 &&
            std::abs(ring[i].x() - ring[0].x()) < 1e-9 &&
            std::abs(ring[i].y() - ring[0].y()) < 1e-9) {
            continue;
        }
        points.push_back(PointDto{ring[i].x(), ring[i].y()});
    }
    return points;
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
    loaded_map->connectivity = costs::ZoneSelector::buildConnectivityIndex(zones);
    loaded_map->response.map_id = nextMapId();
    loaded_map->response.filename = request.filename;
    loaded_map->response.map_name = extractMapName(request.yaml, request.filename);
    loaded_map->response.bounds = computeBounds(zones);
    loaded_map->response.vehicle = buildVehicleFootprintDto(car_definition);

    loaded_map->response.zones.reserve(zones.size());
    for (const auto& zone : zones) {
        loaded_map->response.zones.push_back(buildZoneDto(zone));
    }

    std::lock_guard<std::mutex> lock(maps_mutex_);
    maps_[loaded_map->response.map_id] = loaded_map;
    return loaded_map->response;
}

SearchSpacePreviewResponse PlannerVisualizerService::previewSearchSpace(
    const SearchSpacePreviewRequest& request) {
    if (request.map_id.empty()) {
        throw std::runtime_error("Search-space preview request is missing map_id.");
    }

    const auto loaded_map = requireMap(request.map_id);
    const auto start_pose = makePose(request.start, "start");
    const auto goal_pose = makePose(request.goal, "goal");

    SearchSpacePreviewResponse response;
    try {
        costs::ZoneSelector selector;
        const auto selection = selector.select(
            start_pose,
            goal_pose,
            loaded_map->zones);
        response.success = true;
        response.detail = "Search space resolved.";
        response.search_boundary = polygonToPointDtos(selection.search_boundary);
    } catch (const std::exception& e) {
        response.success = false;
        response.detail = e.what();
    }
    return response;
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

    struct AttemptRecord {
        planning::PlanningAttempt attempt;
        planning::HybridAStarPlannerResult result;
    };

    planning::HybridAStarPlannerResult last_planner_result;
    std::vector<AttemptRecord> attempt_records;
    auto runner = [&](const planning::PlanningAttempt& attempt) {
        planning::HybridAStarPlanner planner(car, loaded_map->zones, behavior_set);
        planning::HybridAStarPlannerRequest planner_request;
        planner_request.start = start_pose;
        planner_request.goal = goal_pose;
        planner_request.initial_behavior_name = attempt.profile;

        last_planner_result = planner.plan(planner_request);
        attempt_records.push_back(AttemptRecord{attempt, last_planner_result});
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
    response.debug_mode = behavior_set.debugModeEnabled();

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

    if (response.debug_mode) {
        try {
            const auto sanitize = [](std::string value) {
                for (char& ch : value) {
                    const bool keep =
                        (ch >= 'a' && ch <= 'z') ||
                        (ch >= 'A' && ch <= 'Z') ||
                        (ch >= '0' && ch <= '9') ||
                        ch == '_' || ch == '-';
                    if (!keep) {
                        ch = '_';
                    }
                }
                return value;
            };

            json attempts = json::array();
            for (const auto& record : attempt_records) {
                json path_json = json::array();
                for (const auto& pose : record.result.poses) {
                    path_json.push_back(poseToJson(pose));
                }

                json segment_dirs = json::array();
                for (const auto direction : record.result.segment_directions) {
                    segment_dirs.push_back(motionDirectionToString(direction));
                }

                json attempt_json{
                    {"attempt_index", record.attempt.attempt_index},
                    {"profile", record.attempt.profile},
                    {"result", {
                        {"success", record.result.success},
                        {"detail", record.result.detail},
                        {"path", path_json},
                        {"behavior_sequence", record.result.behavior_sequence},
                        {"segment_directions", segment_dirs}
                    }}
                };
                if (behavior_set.contains(record.attempt.profile)) {
                    attempt_json["profile_config"] =
                        profileToJson(behavior_set.get(record.attempt.profile));
                }
                if (record.result.debug_trace != nullptr) {
                    attempt_json["debug_trace"] =
                        plannerDebugTraceToJson(*record.result.debug_trace);
                    attempt_json["profiling"] = json{
                        {"scopes", profilingScopesToJson(
                            record.result.debug_trace->profiling_scopes)}
                    };
                } else {
                    attempt_json["debug_trace"] = json(nullptr);
                    attempt_json["profiling"] = json(nullptr);
                }
                attempts.push_back(std::move(attempt_json));
            }

            json final_path = json::array();
            for (const auto& pose : response.path) {
                final_path.push_back(json{
                    {"x", pose.x},
                    {"y", pose.y},
                    {"heading_deg", pose.heading_deg}
                });
            }

            json report{
                {"request", {
                    {"map_id", request.map_id},
                    {"map_name", loaded_map->response.map_name},
                    {"robot_name", selected_robot_name},
                    {"start", poseToJson(start_pose)},
                    {"goal", poseToJson(goal_pose)},
                    {"start_zone", {
                        {"name", start_zone->getName().value_or(zoneTypeName(start_zone))},
                        {"type", zoneTypeName(start_zone)},
                        {"planner_behavior", start_zone->getResolvedPlannerBehavior()}
                    }},
                    {"goal_zone", {
                        {"name", goal_zone->getName().value_or(zoneTypeName(goal_zone))},
                        {"type", zoneTypeName(goal_zone)},
                        {"planner_behavior", goal_zone->getResolvedPlannerBehavior()}
                    }}
                }},
                {"vehicle", {
                    {"name", car_definition.name},
                    {"wheelbase_m", car_definition.wheelbase_m},
                    {"front_overhang_m", car_definition.front_overhang_m},
                    {"rear_overhang_m", car_definition.rear_overhang_m},
                    {"width_m", car_definition.width_m},
                    {"min_turning_radius_m", car_definition.minTurningRadiusMeters()},
                    {"max_steer_angle_rad", car_definition.maxSteerAngleRadians()}
                }},
                {"global", {
                    {"debug_mode", true}
                }},
                {"orchestration", {
                    {"success", orchestration_result.success},
                    {"preferred_profile", orchestration_result.preferred_profile},
                    {"selected_profile", orchestration_result.selected_profile},
                    {"detail", response.detail},
                    {"attempted_profiles", orchestration_result.attempted_profiles}
                }},
                {"final_result", {
                    {"success", response.success},
                    {"selected_profile", response.selected_profile},
                    {"detail", response.detail},
                    {"behavior_sequence", response.behavior_sequence},
                    {"segment_directions", response.segment_directions},
                    {"path", final_path}
                }},
                {"attempts", attempts}
            };

            const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            const std::filesystem::path report_dir =
                config_.configs_root.parent_path() / "planner_debug_reports";
            std::filesystem::create_directories(report_dir);
            const std::filesystem::path report_path =
                report_dir /
                ("planner_debug_" + std::to_string(now_ms) + "_" +
                 sanitize(loaded_map->response.map_name) + "_" +
                 sanitize(selected_robot_name) + ".json");

            std::ofstream report_stream(report_path);
            if (!report_stream.is_open()) {
                throw std::runtime_error(
                    "Unable to open debug report file: " + report_path.string());
            }
            report_stream << report.dump(2);
            response.debug_report_path = report_path.string();
        } catch (const std::exception& e) {
            response.detail += " Debug report write failed: " + std::string(e.what());
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
        detail.find("exhausted the search space") != std::string::npos ||
        detail.find("exhausted the frontier queues") != std::string::npos) {
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
