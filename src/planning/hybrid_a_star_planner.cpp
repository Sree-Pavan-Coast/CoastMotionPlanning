#include "coastmotionplanning/planning/hybrid_a_star_planner.hpp"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>

#include "coastmotionplanning/collision_checking/collision_checker.hpp"
#include "coastmotionplanning/common/math_constants.hpp"
#include "coastmotionplanning/costs/costmap_builder.hpp"
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/costs/zone_selector.hpp"
#include "coastmotionplanning/motion_primitives/car_motion_table.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

namespace coastmotionplanning {
namespace planning {

namespace {

using Clock = std::chrono::steady_clock;
using collision_checking::CollisionChecker;
using collision_checking::CollisionCheckerConfig;
using costs::CostValues;
using costs::DualModelNonHolonomicHeuristic;
using costs::HeuristicModel;
using costs::ZoneSelectionResult;
using motion_primitives::CarMotionTable;
using motion_primitives::MotionTableConfig;
using motion_primitives::TurnDirection;

constexpr double LARGE_COST = 1e9;
constexpr double MOTION_EPSILON = 1e-6;

enum class TurnClass {
    UNKNOWN = 0,
    STRAIGHT,
    LEFT,
    RIGHT
};

struct SearchNode {
    math::Pose2d pose;
    unsigned int heading_bin{0};
    double g{0.0};
    double h{0.0};
    int parent_index{-1};
    std::shared_ptr<zones::Zone> zone;
    std::string behavior_name;
    TurnClass last_turn_class{TurnClass::UNKNOWN};
    bool has_inbound_motion{false};
    common::MotionDirection inbound_motion{common::MotionDirection::Forward};
    double same_motion_length_m{0.0};
    double same_motion_remaining_to_change_m{0.0};
};

struct DiscreteStateKey {
    int x_idx{0};
    int y_idx{0};
    int theta_idx{0};
    int motion_state{0};
    int turn_class{0};
    int same_motion_length_bucket{0};
    int same_motion_remaining_bucket{0};

    bool operator==(const DiscreteStateKey& other) const {
        return x_idx == other.x_idx &&
               y_idx == other.y_idx &&
               theta_idx == other.theta_idx &&
               motion_state == other.motion_state &&
               turn_class == other.turn_class &&
               same_motion_length_bucket == other.same_motion_length_bucket &&
               same_motion_remaining_bucket == other.same_motion_remaining_bucket;
    }
};

struct DiscreteStateKeyHash {
    size_t operator()(const DiscreteStateKey& key) const {
        size_t seed = std::hash<int>()(key.x_idx);
        seed ^= std::hash<int>()(key.y_idx) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.theta_idx) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.motion_state) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.turn_class) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^=
            std::hash<int>()(key.same_motion_length_bucket) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^=
            std::hash<int>()(key.same_motion_remaining_bucket) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct OpenEntry {
    double f{0.0};
    double h{0.0};
    uint64_t insertion_order{0};
    size_t node_index{0};

    bool operator>(const OpenEntry& other) const {
        if (f != other.f) {
            return f > other.f;
        }
        if (h != other.h) {
            return h > other.h;
        }
        return insertion_order > other.insertion_order;
    }
};

struct AnalyticPathValidationResult {
    bool valid{false};
    std::vector<common::MotionDirection> segment_directions;
};

bool canChangeMotionDirection(const SearchNode& node,
                              common::MotionDirection successor_motion);
double computeNextSameMotionLength(const SearchNode& parent,
                                   common::MotionDirection successor_motion,
                                   double travel_m,
                                   double global_max_same_motion_length_m);
double computeNextSameMotionRemainingToChange(
    const SearchNode& parent,
    const PlannerBehaviorProfile& current_profile,
    const PlannerBehaviorProfile& successor_profile,
    common::MotionDirection successor_motion,
    double travel_m);
bool isTerminalMotionSegmentValid(const SearchNode& node);

double normalizeAngleSigned(double angle) {
    angle = std::fmod(angle + common::PI, common::TWO_PI);
    if (angle < 0.0) {
        angle += common::TWO_PI;
    }
    return angle - common::PI;
}

double normalizeAngleUnsigned(double angle) {
    angle = std::fmod(angle, common::TWO_PI);
    if (angle < 0.0) {
        angle += common::TWO_PI;
    }
    return angle;
}

TurnClass classifyTurn(TurnDirection turn_direction) {
    switch (turn_direction) {
        case TurnDirection::FORWARD:
        case TurnDirection::REVERSE:
            return TurnClass::STRAIGHT;
        case TurnDirection::LEFT:
        case TurnDirection::REV_LEFT:
            return TurnClass::LEFT;
        case TurnDirection::RIGHT:
        case TurnDirection::REV_RIGHT:
            return TurnClass::RIGHT;
        case TurnDirection::UNKNOWN:
            return TurnClass::UNKNOWN;
    }
    return TurnClass::UNKNOWN;
}

common::MotionDirection motionDirectionForPrimitive(TurnDirection turn_direction) {
    switch (turn_direction) {
        case TurnDirection::REVERSE:
        case TurnDirection::REV_LEFT:
        case TurnDirection::REV_RIGHT:
            return common::MotionDirection::Reverse;
        case TurnDirection::FORWARD:
        case TurnDirection::LEFT:
        case TurnDirection::RIGHT:
        case TurnDirection::UNKNOWN:
            return common::MotionDirection::Forward;
    }
    return common::MotionDirection::Forward;
}

DiscreteStateKey discretizeState(const math::Pose2d& pose,
                                 unsigned int heading_bin,
                                 const SearchNode& node,
                                 double xy_resolution_m,
                                 double global_max_same_motion_length_m) {
    const auto discretizeMetric = [&](double value) {
        if (global_max_same_motion_length_m <= MOTION_EPSILON ||
            xy_resolution_m <= MOTION_EPSILON) {
            return 0;
        }
        const double capped = std::clamp(value, 0.0, global_max_same_motion_length_m);
        return static_cast<int>(std::lround(capped / xy_resolution_m));
    };

    return DiscreteStateKey{
        static_cast<int>(std::lround(pose.x / xy_resolution_m)),
        static_cast<int>(std::lround(pose.y / xy_resolution_m)),
        static_cast<int>(heading_bin),
        node.has_inbound_motion
            ? (node.inbound_motion == common::MotionDirection::Forward ? 1 : 2)
            : 0,
        static_cast<int>(node.last_turn_class),
        discretizeMetric(node.same_motion_length_m),
        discretizeMetric(node.same_motion_remaining_to_change_m)
    };
}

double computeGlobalMaxSameMotionLength(const PlannerBehaviorSet& behavior_set) {
    double max_length = 0.0;
    for (const auto& behavior_name : behavior_set.names()) {
        max_length = std::max(
            max_length,
            behavior_set.get(behavior_name).planner.min_path_len_in_same_motion);
    }
    return max_length;
}

robot::RobotState makeRobotState(const math::Pose2d& pose) {
    robot::RobotState state;
    state.x = pose.x;
    state.y = pose.y;
    state.yaw = pose.theta.radians();
    return state;
}

double elapsedMilliseconds(const Clock::time_point& start,
                           const Clock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string zoneLabel(const std::shared_ptr<zones::Zone>& zone) {
    if (zone == nullptr) {
        return "";
    }
    if (zone->getName().has_value() && !zone->getName()->empty()) {
        return zone->getName().value();
    }
    if (std::dynamic_pointer_cast<zones::TrackMainRoad>(zone) != nullptr) {
        return "TrackMainRoad";
    }
    if (std::dynamic_pointer_cast<zones::ManeuveringZone>(zone) != nullptr) {
        return "ManeuveringZone";
    }
    return "Zone";
}

std::string turnDirectionName(TurnDirection turn_direction) {
    switch (turn_direction) {
    case TurnDirection::FORWARD:
        return "FORWARD";
    case TurnDirection::LEFT:
        return "LEFT";
    case TurnDirection::RIGHT:
        return "RIGHT";
    case TurnDirection::REVERSE:
        return "REVERSE";
    case TurnDirection::REV_LEFT:
        return "REV_LEFT";
    case TurnDirection::REV_RIGHT:
        return "REV_RIGHT";
    case TurnDirection::UNKNOWN:
        return "UNKNOWN";
    }
    return "UNKNOWN";
}

double readLayerCost(const grid_map::GridMap& costmap,
                     const std::string& layer,
                     const grid_map::Position& position) {
    if (!costmap.exists(layer) || !costmap.isInside(position)) {
        return 0.0;
    }

    const float value = costmap.atPosition(
        layer, position, grid_map::InterpolationMethods::INTER_NEAREST);
    if (std::isnan(value)) {
        return 0.0;
    }
    return static_cast<double>(value);
}

double readRawLayerValue(const grid_map::GridMap& costmap,
                         const std::string& layer,
                         const grid_map::Position& position) {
    if (!costmap.exists(layer) || !costmap.isInside(position)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return static_cast<double>(costmap.atPosition(
        layer, position, grid_map::InterpolationMethods::INTER_NEAREST));
}

double computeHolonomicHeuristic(const grid_map::GridMap& costmap,
                                 const grid_map::Position& position,
                                 common::ProfilingCollector* profiler = nullptr) {
    common::ScopedProfilingTimer timer(profiler, "planner.holonomic_heuristic_read");
    if (!costmap.exists(costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES) ||
        !costmap.isInside(position)) {
        return LARGE_COST;
    }

    const float value = costmap.atPosition(
        costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES,
        position,
        grid_map::InterpolationMethods::INTER_NEAREST);
    if (std::isnan(value)) {
        return LARGE_COST;
    }
    return static_cast<double>(value);
}

double computeNonHolonomicHeuristic(const DualModelNonHolonomicHeuristic& heuristic,
                                    HeuristicModel model,
                                    const math::Pose2d& from,
                                    const math::Pose2d& goal,
                                    common::ProfilingCollector* profiler = nullptr) {
    common::ScopedProfilingTimer timer(profiler, "planner.non_holonomic_heuristic_lookup");
    const double dx_world = goal.x - from.x;
    const double dy_world = goal.y - from.y;
    const double cos_yaw = std::cos(from.theta.radians());
    const double sin_yaw = std::sin(from.theta.radians());

    const double rel_x = cos_yaw * dx_world + sin_yaw * dy_world;
    const double rel_y = -sin_yaw * dx_world + cos_yaw * dy_world;
    const double rel_theta = normalizeAngleSigned(goal.theta.radians() - from.theta.radians());

    return heuristic.lookup(
        model,
        static_cast<float>(rel_x),
        static_cast<float>(rel_y),
        static_cast<float>(rel_theta));
}

double computeEdgeCost(const grid_map::GridMap& costmap,
                       const PlannerBehaviorProfile& profile,
                       const SearchNode& parent,
                       const grid_map::Position& successor_position,
                       double successor_yaw_rad,
                       TurnDirection turn_direction,
                       double travel_m) {
    double edge_cost = travel_m;
    const common::MotionDirection successor_motion =
        motionDirectionForPrimitive(turn_direction);
    const bool reverse_motion = successor_motion == common::MotionDirection::Reverse;

    edge_cost *= reverse_motion
        ? profile.planner.weight_reverse
        : profile.planner.weight_forward;

    const TurnClass turn_class = classifyTurn(turn_direction);
    if (turn_class == TurnClass::LEFT || turn_class == TurnClass::RIGHT) {
        edge_cost += profile.planner.weight_steer * travel_m;
    }

    if (parent.last_turn_class != TurnClass::UNKNOWN &&
        parent.last_turn_class != turn_class) {
        edge_cost += profile.planner.weight_steer_change * travel_m;
    }

    if (parent.has_inbound_motion && parent.inbound_motion != successor_motion) {
        edge_cost += profile.planner.weight_gear_change;
    }

    if (profile.isLayerActive(costs::CostmapLayerNames::INFLATION)) {
        edge_cost += travel_m * (readLayerCost(costmap, costs::CostmapLayerNames::INFLATION,
                                               successor_position) / CostValues::LETHAL);
    }

    if (profile.isLayerActive(costs::CostmapLayerNames::LANE_CENTERLINE_COST)) {
        edge_cost += travel_m * profile.planner.weight_lane_centerline *
            (readLayerCost(costmap,
                           costs::CostmapLayerNames::LANE_CENTERLINE_COST,
                           successor_position) / CostValues::LETHAL);
    }

    if (profile.planner.lane_heading_bias_weight > 0.0) {
        const double lane_heading = readRawLayerValue(
            costmap, costs::CostmapLayerNames::LANE_HEADING, successor_position);
        if (std::isfinite(lane_heading)) {
            const double heading_error =
                std::abs(normalizeAngleSigned(successor_yaw_rad - lane_heading));
            edge_cost +=
                travel_m * profile.planner.lane_heading_bias_weight * heading_error;
        }
    }

    return edge_cost;
}

bool canChangeMotionDirection(const SearchNode& node,
                              common::MotionDirection successor_motion) {
    return !node.has_inbound_motion ||
           node.inbound_motion == successor_motion ||
           node.same_motion_remaining_to_change_m <= MOTION_EPSILON;
}

double computeNextSameMotionLength(const SearchNode& parent,
                                   common::MotionDirection successor_motion,
                                   double travel_m,
                                   double global_max_same_motion_length_m) {
    const double unclamped_length =
        (!parent.has_inbound_motion || parent.inbound_motion != successor_motion)
        ? travel_m
        : parent.same_motion_length_m + travel_m;
    if (global_max_same_motion_length_m <= MOTION_EPSILON) {
        return 0.0;
    }
    return std::min(unclamped_length, global_max_same_motion_length_m);
}

double computeNextSameMotionRemainingToChange(
    const SearchNode& parent,
    const PlannerBehaviorProfile& current_profile,
    const PlannerBehaviorProfile& successor_profile,
    common::MotionDirection successor_motion,
    double travel_m) {
    if (!parent.has_inbound_motion || parent.inbound_motion != successor_motion) {
        const double new_segment_min = std::max(
            current_profile.planner.min_path_len_in_same_motion,
            successor_profile.planner.min_path_len_in_same_motion);
        return std::max(0.0, new_segment_min - travel_m);
    }

    return std::max(
        0.0,
        std::max(
            parent.same_motion_remaining_to_change_m,
            successor_profile.planner.min_path_len_in_same_motion -
                parent.same_motion_length_m) -
            travel_m);
}

bool isTerminalMotionSegmentValid(const SearchNode& node) {
    return !node.has_inbound_motion ||
           node.same_motion_remaining_to_change_m <= MOTION_EPSILON;
}

bool allowsReverseMotion(const PlannerBehaviorProfile& profile,
                         const std::shared_ptr<zones::Zone>& zone) {
    return !profile.planner.only_forward_path &&
           (zone == nullptr || zone->isReverseAllowed());
}

bool allowsReverseMotion(const ResolvedPlannerBehavior& resolved_behavior) {
    return resolved_behavior.profile != nullptr &&
           allowsReverseMotion(*resolved_behavior.profile, resolved_behavior.zone);
}

common::MotionDirection inferAnalyticMotionDirection(
    const std::array<double, 3>& from,
    const std::array<double, 3>& to,
    common::MotionDirection fallback_direction) {
    const double dx = to[0] - from[0];
    const double dy = to[1] - from[1];
    const double distance = std::hypot(dx, dy);
    if (distance <= MOTION_EPSILON) {
        return fallback_direction;
    }

    const double start_projection = dx * std::cos(from[2]) + dy * std::sin(from[2]);
    const double end_projection = dx * std::cos(to[2]) + dy * std::sin(to[2]);
    const double mean_projection = 0.5 * (start_projection + end_projection);
    const double projection_threshold = std::max(MOTION_EPSILON, distance * 1e-3);
    if (mean_projection > projection_threshold) {
        return common::MotionDirection::Forward;
    }
    if (mean_projection < -projection_threshold) {
        return common::MotionDirection::Reverse;
    }
    return fallback_direction;
}

AnalyticPathValidationResult validateAnalyticPathSegments(
    const SearchNode& start_node,
    const std::vector<std::array<double, 3>>& waypoints,
    const std::vector<ResolvedPlannerBehavior>& waypoint_resolutions,
    double global_max_same_motion_length_m) {
    AnalyticPathValidationResult result;
    if (waypoints.size() < 2 || waypoints.size() != waypoint_resolutions.size()) {
        return result;
    }

    SearchNode simulated_node = start_node;
    common::MotionDirection fallback_direction = simulated_node.has_inbound_motion
        ? simulated_node.inbound_motion
        : common::MotionDirection::Forward;
    result.segment_directions.reserve(waypoints.size() - 1);

    for (size_t i = 1; i < waypoints.size(); ++i) {
        const auto& from = waypoints[i - 1];
        const auto& to = waypoints[i];
        const ResolvedPlannerBehavior& current_resolution = waypoint_resolutions[i - 1];
        const ResolvedPlannerBehavior& successor_resolution = waypoint_resolutions[i];
        if (current_resolution.profile == nullptr || successor_resolution.profile == nullptr) {
            return AnalyticPathValidationResult{};
        }

        const double travel_m = std::hypot(to[0] - from[0], to[1] - from[1]);
        const common::MotionDirection step_motion = inferAnalyticMotionDirection(
            from, to, fallback_direction);

        if (step_motion == common::MotionDirection::Reverse &&
            !allowsReverseMotion(*successor_resolution.profile, successor_resolution.zone)) {
            return AnalyticPathValidationResult{};
        }

        if (!canChangeMotionDirection(simulated_node, step_motion)) {
            return AnalyticPathValidationResult{};
        }

        simulated_node.same_motion_length_m = computeNextSameMotionLength(
            simulated_node,
            step_motion,
            travel_m,
            global_max_same_motion_length_m);
        simulated_node.same_motion_remaining_to_change_m =
            computeNextSameMotionRemainingToChange(
                simulated_node,
                *current_resolution.profile,
                *successor_resolution.profile,
                step_motion,
                travel_m);
        simulated_node.has_inbound_motion = true;
        simulated_node.inbound_motion = step_motion;
        simulated_node.zone = successor_resolution.zone;
        simulated_node.behavior_name = successor_resolution.behavior_name;
        simulated_node.pose = math::Pose2d(
            to[0], to[1], math::Angle::from_radians(to[2]));

        fallback_direction = step_motion;
        result.segment_directions.push_back(step_motion);
    }

    result.valid = isTerminalMotionSegmentValid(simulated_node);
    if (!result.valid) {
        result.segment_directions.clear();
    }
    return result;
}

bool isGoalSatisfied(const SearchNode& node,
                     const math::Pose2d& goal,
                     double goal_distance_tolerance,
                     double yaw_bin_size_rad) {
    const double dx = goal.x - node.pose.x;
    const double dy = goal.y - node.pose.y;
    const double distance = std::hypot(dx, dy);
    const double heading_error =
        std::abs(normalizeAngleSigned(goal.theta.radians() - node.pose.theta.radians()));

    return distance <= goal_distance_tolerance &&
           heading_error <= yaw_bin_size_rad;
}

HybridAStarPlannerResult reconstructResult(const std::vector<SearchNode>& nodes,
                                           int goal_index,
                                           common::ProfilingCollector* profiler = nullptr) {
    common::ScopedProfilingTimer timer(profiler, "planner.result_reconstruction");
    HybridAStarPlannerResult result;
    result.success = true;

    std::vector<math::Pose2d> reversed_poses;
    std::vector<std::string> reversed_behaviors;
    std::vector<common::MotionDirection> reversed_directions;

    int current_index = goal_index;
    while (current_index >= 0) {
        const SearchNode& node = nodes[static_cast<size_t>(current_index)];
        reversed_poses.push_back(node.pose);
        reversed_behaviors.push_back(node.behavior_name);
        if (node.parent_index >= 0) {
            reversed_directions.push_back(node.inbound_motion);
        }
        current_index = node.parent_index;
    }

    result.poses.assign(reversed_poses.rbegin(), reversed_poses.rend());
    result.behavior_sequence.assign(reversed_behaviors.rbegin(), reversed_behaviors.rend());
    result.segment_directions.assign(reversed_directions.rbegin(), reversed_directions.rend());
    result.detail = "Path found.";
    return result;
}

} // namespace

HybridAStarPlanner::HybridAStarPlanner(
    const robot::Car& car,
    std::vector<std::shared_ptr<zones::Zone>> all_zones,
    PlannerBehaviorSet behavior_set)
    : car_(car),
      all_zones_(std::move(all_zones)),
      behavior_set_(std::move(behavior_set)) {}

HybridAStarPlannerResult HybridAStarPlanner::plan(
    const HybridAStarPlannerRequest& request) const {
    const Clock::time_point plan_start_time = Clock::now();
    HybridAStarPlannerResult failure_result;
    const std::shared_ptr<HybridAStarPlannerDebugTrace> debug_trace =
        behavior_set_.debugModeEnabled()
        ? std::make_shared<HybridAStarPlannerDebugTrace>()
        : nullptr;
    common::ProfilingCollector profiling_collector;
    common::ProfilingCollector* profiler =
        debug_trace != nullptr ? &profiling_collector : nullptr;
    size_t debug_nodes_allocated = 0;
    size_t debug_unique_state_count = 0;
    size_t debug_open_queue_peak_size = 0;
    Clock::time_point search_loop_start_time{};
    bool search_loop_started = false;

    if (debug_trace != nullptr) {
        debug_trace->initial_behavior_name = request.initial_behavior_name;
    }

    const auto finalizeResult =
        [&](HybridAStarPlannerResult result) -> HybridAStarPlannerResult {
        if (debug_trace != nullptr) {
            if (search_loop_started) {
                debug_trace->search_loop_ms =
                    elapsedMilliseconds(search_loop_start_time, Clock::now());
            }
            debug_trace->nodes_allocated = debug_nodes_allocated;
            debug_trace->unique_state_count = debug_unique_state_count;
            debug_trace->open_queue_peak_size = debug_open_queue_peak_size;
            debug_trace->total_planning_ms =
                elapsedMilliseconds(plan_start_time, Clock::now());
            debug_trace->profiling_scopes =
                profiling_collector.snapshotSortedByTotalMs();
            result.debug_trace = debug_trace;
        }
        return result;
    };

    const auto failWithDetail =
        [&](std::string detail) -> HybridAStarPlannerResult {
        failure_result.detail = std::move(detail);
        if (debug_trace != nullptr) {
            debug_trace->terminal_reason = failure_result.detail;
        }
        return finalizeResult(failure_result);
    };

    if (!behavior_set_.contains(request.initial_behavior_name)) {
        return failWithDetail(
            "Initial planner behavior '" + request.initial_behavior_name + "' is not defined.");
    }

    const PlannerBehaviorProfile& initial_profile =
        behavior_set_.get(request.initial_behavior_name);
    if (initial_profile.motion_primitives.min_turning_radius_m <= 0.0 ||
        initial_profile.motion_primitives.max_steer_angle_rad <= 0.0) {
        return failWithDetail(
            "Motion primitive constraints were not initialized from the selected robot model.");
    }

    ZoneSelectionResult selection;
    try {
        const Clock::time_point selection_start_time = Clock::now();
        common::ScopedProfilingTimer selection_timer(profiler, "planner.zone_selection");
        costs::ZoneSelector selector;
        selection = selector.select(
            request.start,
            request.goal,
            all_zones_,
            initial_profile.costmap.alpha_shape_alpha);
        if (debug_trace != nullptr) {
            debug_trace->zone_selection_ms =
                elapsedMilliseconds(selection_start_time, Clock::now());
            for (const auto& zone : selection.selected_zones) {
                debug_trace->selected_zone_names.push_back(zoneLabel(zone));
            }
        }
    } catch (const std::exception& e) {
        return failWithDetail(e.what());
    }

    const auto start_zone = costs::ZoneSelector::findContainingZone(
        geometry::Point2d(request.start.x, request.start.y), selection.selected_zones);
    const auto goal_zone = costs::ZoneSelector::findContainingZone(
        geometry::Point2d(request.goal.x, request.goal.y), selection.selected_zones);
    if (debug_trace != nullptr) {
        debug_trace->start_zone_name = zoneLabel(start_zone);
        debug_trace->goal_zone_name = zoneLabel(goal_zone);
    }
    if (start_zone == nullptr) {
        return failWithDetail("Start pose is not inside the selected planning zones.");
    }

    costs::CostmapBuilder builder(
        initial_profile.makeCostmapConfig(), all_zones_, car_, profiler);
    const Clock::time_point costmap_start_time = Clock::now();
    grid_map::GridMap costmap = builder.build(selection, request.goal);
    if (debug_trace != nullptr) {
        debug_trace->costmap_build_ms =
            elapsedMilliseconds(costmap_start_time, Clock::now());
    }

    CollisionCheckerConfig collision_config;
    collision_config.obstacle_layer = costs::CostmapLayerNames::COMBINED_COST;
    collision_config.lethal_threshold =
        static_cast<float>(initial_profile.collision_checker.lethal_threshold);
    CollisionChecker collision_checker(collision_config);

    const bool start_in_collision = [&]() {
        common::ScopedProfilingTimer timer(
            profiler, "planner.collision_check.start_validation");
        return collision_checker.checkCollision(
            costmap, car_, makeRobotState(request.start)).in_collision;
    }();
    if (start_in_collision) {
        return failWithDetail("Start pose is in collision on the selected-zone costmap.");
    }
    const bool goal_in_collision = [&]() {
        common::ScopedProfilingTimer timer(
            profiler, "planner.collision_check.goal_validation");
        return collision_checker.checkCollision(
            costmap, car_, makeRobotState(request.goal)).in_collision;
    }();
    if (goal_in_collision) {
        return failWithDetail("Goal pose is in collision on the selected-zone costmap.");
    }

    MotionTableConfig motion_config;
    motion_config.minimum_turning_radius = static_cast<float>(
        initial_profile.motion_primitives.min_turning_radius_m /
        initial_profile.planner.xy_grid_resolution_m);
    motion_config.num_angle_quantization = static_cast<unsigned int>(
        initial_profile.motion_primitives.num_angle_bins);

    const Clock::time_point heuristic_setup_start_time = Clock::now();
    CarMotionTable motion_table;
    motion_table.initReedsShepp(motion_config);

    DualModelNonHolonomicHeuristic heuristic;
    heuristic.configureOmpl(
        static_cast<float>(initial_profile.motion_primitives.min_turning_radius_m));
    if (!request.dual_model_lut_path.empty()) {
        heuristic.loadFromFile(request.dual_model_lut_path);
    }
    if (debug_trace != nullptr) {
        debug_trace->heuristic_setup_ms =
            elapsedMilliseconds(heuristic_setup_start_time, Clock::now());
    }

    const double xy_resolution_m = initial_profile.planner.xy_grid_resolution_m;
    const double yaw_bin_size_rad = motion_table.getBinSize();
    const double goal_distance_tolerance = initial_profile.planner.step_size_m;
    const double global_max_same_motion_length_m =
        computeGlobalMaxSameMotionLength(behavior_set_);
    const auto deadline = Clock::now() + std::chrono::milliseconds(
        initial_profile.planner.max_planning_time_ms);
    search_loop_start_time = Clock::now();
    search_loop_started = true;

    // Analytic expansion parameters
    const double analytic_max_length = initial_profile.planner.analytic_expansion_max_length_m;
    const double analytic_ratio = initial_profile.planner.analytic_expansion_ratio;
    const unsigned int analytic_expansion_interval = std::max(
        1u, static_cast<unsigned int>(
            std::round(1.0 / std::max(analytic_ratio, 0.01))));
    const double analytic_step_size = xy_resolution_m * 2.0;

    const double normalized_start_yaw = normalizeAngleUnsigned(request.start.theta.radians());
    const unsigned int start_heading_bin =
        motion_table.getClosestAngularBin(normalized_start_yaw);
    const HeuristicModel start_model = allowsReverseMotion(initial_profile, start_zone)
        ? HeuristicModel::REEDS_SHEPP
        : HeuristicModel::DUBINS;
    const double start_h = std::max(
        computeHolonomicHeuristic(
            costmap,
            grid_map::Position(request.start.x, request.start.y),
            profiler),
        computeNonHolonomicHeuristic(
            heuristic, start_model, request.start, request.goal, profiler));

    SearchNode start_node;
    start_node.pose = request.start;
    start_node.heading_bin = start_heading_bin;
    start_node.g = 0.0;
    start_node.h = start_h;
    start_node.zone = start_zone;
    start_node.behavior_name = request.initial_behavior_name;

    std::vector<SearchNode> nodes;
    nodes.reserve(50000);
    nodes.push_back(start_node);
    debug_nodes_allocated = nodes.size();

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open_queue;
    open_queue.push(OpenEntry{start_node.g + start_node.h, start_node.h, 0, 0});
    debug_open_queue_peak_size = std::max(debug_open_queue_peak_size, open_queue.size());

    std::unordered_map<DiscreteStateKey, double, DiscreteStateKeyHash> best_g_by_key;
    best_g_by_key.reserve(50000);
    best_g_by_key.emplace(
        discretizeState(
            request.start,
            start_heading_bin,
            start_node,
            xy_resolution_m,
            global_max_same_motion_length_m),
        0.0);
    debug_unique_state_count = best_g_by_key.size();

    uint64_t insertion_order = 1;
    uint64_t expansion_count = 0;
    struct AnalyticAttemptOutcome {
        HybridAStarPlannerResult result;
        PlannerAnalyticDebugEvent debug_event;
    };

    // Lambda: attempt analytic expansion from a node to the goal.
    // Returns a result with success=true if a collision-free path is found.
    auto tryAnalyticExpansion = [&](const SearchNode& node,
                                   size_t node_index,
                                   const std::string& trigger) -> AnalyticAttemptOutcome {
        common::ScopedProfilingTimer analytic_timer(
            profiler, "planner.analytic_expansion_attempt");
        AnalyticAttemptOutcome outcome;
        outcome.debug_event.attempted = true;
        outcome.debug_event.trigger = trigger;
        outcome.debug_event.distance_to_goal_m =
            std::hypot(request.goal.x - node.pose.x, request.goal.y - node.pose.y);
        if (!heuristic.hasOmplSpaces()) {
            outcome.debug_event.outcome = "failed";
            outcome.debug_event.detail = "no_ompl_spaces";
            return outcome;
        }

        // Don't expand if the node hasn't satisfied its minimum same-motion run
        if (node.same_motion_remaining_to_change_m > MOTION_EPSILON) {
            outcome.debug_event.outcome = "failed";
            outcome.debug_event.detail = "same_motion_guard";
            return outcome;
        }

        const PlannerBehaviorProfile& node_profile =
            behavior_set_.get(node.behavior_name);
        const HeuristicModel model =
            allowsReverseMotion(node_profile, node.zone)
            ? HeuristicModel::REEDS_SHEPP
            : HeuristicModel::DUBINS;

        std::pair<double, std::vector<std::array<double, 3>>> sampled_path;
        {
            common::ScopedProfilingTimer sample_path_timer(
                profiler, "planner.analytic_sample_path");
            sampled_path = heuristic.samplePath(
                model,
                node.pose.x, node.pose.y, node.pose.theta.radians(),
                request.goal.x, request.goal.y, request.goal.theta.radians(),
                analytic_step_size);
        }
        auto& path_length = sampled_path.first;
        auto& waypoints = sampled_path.second;
        outcome.debug_event.path_length_m = path_length;
        outcome.debug_event.waypoint_count = waypoints.size();

        if (waypoints.empty() || path_length > analytic_max_length) {
            outcome.debug_event.outcome = "failed";
            outcome.debug_event.detail =
                waypoints.empty() ? "no_waypoints" : "path_length_exceeded";
            return outcome;
        }

        // Check all waypoints for collision and resolve behaviors
        std::vector<ResolvedPlannerBehavior> wp_resolutions;
        wp_resolutions.reserve(waypoints.size());
        wp_resolutions.push_back(ResolvedPlannerBehavior{
            node.zone, node.behavior_name, &node_profile, false});
        for (size_t i = 1; i < waypoints.size(); ++i) {
            const auto& wp = waypoints[i];
            robot::RobotState rs;
            rs.x = wp[0];
            rs.y = wp[1];
            rs.yaw = wp[2];

            const grid_map::Position wp_pos(rs.x, rs.y);
            if (!costmap.isInside(wp_pos)) {
                outcome.debug_event.outcome = "failed";
                outcome.debug_event.detail = "waypoint_out_of_bounds";
                return outcome;
            }

            const bool analytic_collision = [&]() {
                common::ScopedProfilingTimer collision_timer(
                    profiler, "planner.collision_check.analytic");
                return collision_checker.checkCollision(costmap, car_, rs).in_collision;
            }();
            if (analytic_collision) {
                outcome.debug_event.outcome = "failed";
                outcome.debug_event.detail = "waypoint_collision";
                return outcome;
            }

            // Resolve zone/behavior for this waypoint
            math::Pose2d wp_pose(wp[0], wp[1],
                                 math::Angle::from_radians(wp[2]));
            ResolvedPlannerBehavior resolved;
            {
                common::ScopedProfilingTimer behavior_timer(
                    profiler, "planner.behavior_resolution");
                resolved = PlannerBehaviorResolver::resolve(
                    wp_pose, costmap, node.zone, node.behavior_name,
                    selection.selected_zones, behavior_set_);
            }
            wp_resolutions.push_back(
                resolved.profile != nullptr
                    ? resolved
                    : ResolvedPlannerBehavior{
                          node.zone, node.behavior_name, &node_profile, false});
        }

        const AnalyticPathValidationResult analytic_validation =
            validateAnalyticPathSegments(
                node,
                waypoints,
                wp_resolutions,
                global_max_same_motion_length_m);
        if (!analytic_validation.valid) {
            outcome.debug_event.outcome = "failed";
            outcome.debug_event.detail = "path_validation_failed";
            return outcome;
        }

        common::ScopedProfilingTimer reconstruction_timer(
            profiler, "planner.result_reconstruction");
        // Build the result: search path to current node + analytic path to goal
        HybridAStarPlannerResult result;
        result.success = true;

        // Backtrack from current node to start
        std::vector<math::Pose2d> prefix_poses;
        std::vector<std::string> prefix_behaviors;
        std::vector<common::MotionDirection> prefix_directions;

        int idx = static_cast<int>(node_index);
        while (idx >= 0) {
            const SearchNode& n = nodes[static_cast<size_t>(idx)];
            prefix_poses.push_back(n.pose);
            prefix_behaviors.push_back(n.behavior_name);
            if (n.parent_index >= 0) {
                prefix_directions.push_back(n.inbound_motion);
            }
            idx = n.parent_index;
        }

        std::reverse(prefix_poses.begin(), prefix_poses.end());
        std::reverse(prefix_behaviors.begin(), prefix_behaviors.end());
        std::reverse(prefix_directions.begin(), prefix_directions.end());

        // Append analytic expansion waypoints (skip first, it's the current node)
        for (size_t i = 1; i < waypoints.size(); ++i) {
            math::Pose2d wp_pose;
            wp_pose.x = waypoints[i][0];
            wp_pose.y = waypoints[i][1];
            wp_pose.theta = math::Angle::from_radians(waypoints[i][2]);
            prefix_poses.push_back(wp_pose);
            prefix_behaviors.push_back(wp_resolutions[i].behavior_name);
            prefix_directions.push_back(analytic_validation.segment_directions[i - 1]);
        }

        result.poses = std::move(prefix_poses);
        result.behavior_sequence = std::move(prefix_behaviors);
        result.segment_directions = std::move(prefix_directions);
        result.detail = "Path found (analytic expansion).";
        outcome.result = std::move(result);
        outcome.debug_event.outcome = "success";
        return outcome;
    };

    while (!open_queue.empty()) {
        if (Clock::now() >= deadline) {
            if (debug_trace != nullptr) {
                debug_trace->terminal_reason =
                    "Hybrid A* search timed out before finding a path.";
            }
            return failWithDetail("Hybrid A* search timed out before finding a path.");
        }

        const OpenEntry entry = open_queue.top();
        open_queue.pop();
        if (debug_trace != nullptr) {
            ++debug_trace->expansions_popped;
        }

        // Copy, not reference: nodes.push_back() in the inner loop can
        // trigger reallocation which would invalidate a reference.
        const SearchNode current_node = nodes[entry.node_index];
        const DiscreteStateKey current_key = discretizeState(
            current_node.pose,
            current_node.heading_bin,
            current_node,
            xy_resolution_m,
            global_max_same_motion_length_m);
        const auto best_it = best_g_by_key.find(current_key);
        if (best_it == best_g_by_key.end() || current_node.g > best_it->second + 1e-6) {
            if (debug_trace != nullptr) {
                ++debug_trace->stale_entries_skipped;
            }
            continue;
        }

        PlannerExpansionDebugEvent expansion_debug;
        if (debug_trace != nullptr) {
            expansion_debug.expansion_index = expansion_count;
            expansion_debug.node_index = entry.node_index;
            expansion_debug.pose = current_node.pose;
            expansion_debug.zone_name = zoneLabel(current_node.zone);
            expansion_debug.behavior_name = current_node.behavior_name;
            expansion_debug.g = current_node.g;
            expansion_debug.h = current_node.h;
            expansion_debug.f = entry.f;
            expansion_debug.distance_to_goal_m =
                std::hypot(request.goal.x - current_node.pose.x,
                           request.goal.y - current_node.pose.y);
            expansion_debug.open_queue_size_after_pop = open_queue.size();
        }

        bool goal_satisfied = false;
        bool terminal_motion_valid = false;
        {
            common::ScopedProfilingTimer goal_check_timer(
                profiler, "planner.goal_check");
            goal_satisfied = isGoalSatisfied(current_node,
                                            request.goal,
                                            goal_distance_tolerance,
                                            yaw_bin_size_rad);
            terminal_motion_valid = isTerminalMotionSegmentValid(current_node);
        }
        if (debug_trace != nullptr) {
            ++debug_trace->goal_checks;
            expansion_debug.goal_satisfied = goal_satisfied;
            expansion_debug.terminal_motion_valid = terminal_motion_valid;
        }
        if (goal_satisfied && terminal_motion_valid) {
            if (debug_trace != nullptr) {
                ++debug_trace->goal_hits;
                debug_trace->expansions.push_back(std::move(expansion_debug));
                debug_trace->terminal_reason = "goal_satisfied";
            }
            return finalizeResult(
                reconstructResult(nodes, static_cast<int>(entry.node_index), profiler));
        }

        const PlannerBehaviorProfile& current_profile =
            behavior_set_.get(current_node.behavior_name);

        // Analytic expansion: periodically try a direct curve to the goal
        ++expansion_count;
        const double dx_to_goal = request.goal.x - current_node.pose.x;
        const double dy_to_goal = request.goal.y - current_node.pose.y;
        const double distance_to_goal = std::hypot(dx_to_goal, dy_to_goal);
        const bool periodic_analytic_shot =
            current_profile.planner.analytic_shot &&
            expansion_count % analytic_expansion_interval == 0;
        const bool near_goal_analytic_shot =
            current_profile.planner.near_goal_analytic_expansion &&
            current_profile.planner.near_goal_analytic_radius_m > 0.0 &&
            distance_to_goal <= current_profile.planner.near_goal_analytic_radius_m;
        if (periodic_analytic_shot || near_goal_analytic_shot) {
            const std::string analytic_trigger =
                periodic_analytic_shot && near_goal_analytic_shot
                ? "periodic_and_near_goal"
                : (periodic_analytic_shot ? "periodic" : "near_goal");
            const auto analytic_outcome =
                tryAnalyticExpansion(current_node, entry.node_index, analytic_trigger);
            if (debug_trace != nullptr) {
                ++debug_trace->analytic_attempts;
                expansion_debug.analytic_attempted = true;
                expansion_debug.analytic_event = analytic_outcome.debug_event;
                if (analytic_outcome.result.success) {
                    ++debug_trace->analytic_successes;
                } else if (analytic_outcome.debug_event.detail == "no_ompl_spaces") {
                    ++debug_trace->analytic_fail_no_ompl;
                } else if (analytic_outcome.debug_event.detail == "same_motion_guard") {
                    ++debug_trace->analytic_fail_same_motion_guard;
                } else if (analytic_outcome.debug_event.detail == "path_length_exceeded" ||
                           analytic_outcome.debug_event.detail == "no_waypoints") {
                    ++debug_trace->analytic_fail_path_length;
                } else if (analytic_outcome.debug_event.detail == "waypoint_out_of_bounds") {
                    ++debug_trace->analytic_fail_out_of_bounds;
                } else if (analytic_outcome.debug_event.detail == "waypoint_collision") {
                    ++debug_trace->analytic_fail_collision;
                } else {
                    ++debug_trace->analytic_fail_validation;
                }
            }
            if (analytic_outcome.result.success) {
                if (debug_trace != nullptr) {
                    debug_trace->expansions.push_back(std::move(expansion_debug));
                    debug_trace->terminal_reason = "analytic_expansion_success";
                }
                return finalizeResult(analytic_outcome.result);
            }
        }

        const auto projections = [&]() {
            common::ScopedProfilingTimer projections_timer(
                profiler, "planner.motion_table_projections");
            return motion_table.getProjections(
                static_cast<float>(current_node.pose.x / xy_resolution_m),
                static_cast<float>(current_node.pose.y / xy_resolution_m),
                current_node.heading_bin);
        }();

        bool lane_following_node = false;
        size_t forward_primitive_idx = projections.size();
        if (current_profile.planner.lane_primitive_suppression) {
            const grid_map::Position current_position(current_node.pose.x, current_node.pose.y);
            const double current_lane_distance = readRawLayerValue(
                costmap, costs::CostmapLayerNames::LANE_DISTANCE, current_position);
            const double current_lane_heading = readRawLayerValue(
                costmap, costs::CostmapLayerNames::LANE_HEADING, current_position);
            if (std::isfinite(current_lane_distance) &&
                std::isfinite(current_lane_heading) &&
                current_lane_distance < current_profile.planner.step_size_m) {
                const double heading_error = std::abs(normalizeAngleSigned(
                    current_node.pose.theta.radians() - current_lane_heading));
                if (heading_error < (2.0 * yaw_bin_size_rad)) {
                    const auto forward_it = std::find_if(
                        projections.begin(),
                        projections.end(),
                        [](const auto& projection) {
                            return projection.turn_dir == TurnDirection::FORWARD;
                        });
                    if (forward_it != projections.end()) {
                        lane_following_node = true;
                        forward_primitive_idx = static_cast<size_t>(
                            std::distance(projections.begin(), forward_it));
                        if (debug_trace != nullptr) {
                            ++debug_trace->lane_following_candidates;
                            expansion_debug.lane_following_candidate = true;
                        }
                    }
                }
            }
        }

        const auto recordPrimitiveEvent = [&](PlannerPrimitiveDebugEvent primitive_event) {
            if (debug_trace == nullptr) {
                return;
            }
            if (primitive_event.outcome == "out_of_bounds") {
                ++debug_trace->primitive_out_of_bounds;
            } else if (primitive_event.outcome == "cross_track_pruned") {
                ++debug_trace->primitive_cross_track_pruned;
            } else if (primitive_event.outcome == "behavior_unresolved") {
                ++debug_trace->primitive_behavior_unresolved;
            } else if (primitive_event.outcome == "primitive_disallowed") {
                ++debug_trace->primitive_disallowed;
            } else if (primitive_event.outcome == "collision") {
                ++debug_trace->primitive_collision;
            } else if (primitive_event.outcome == "motion_change_blocked") {
                ++debug_trace->primitive_motion_change_blocked;
            } else if (primitive_event.outcome == "dominated") {
                ++debug_trace->primitive_dominated;
            } else if (primitive_event.outcome == "enqueued") {
                ++debug_trace->primitive_enqueued;
            }
            expansion_debug.primitive_events.push_back(std::move(primitive_event));
        };

        const auto tryExpandPrimitive = [&](size_t primitive_idx) -> bool {
            common::ScopedProfilingTimer primitive_timer(
                profiler, "planner.primitive_expansion_attempt");
            const auto& projection = projections[primitive_idx];
            const unsigned int successor_heading_bin =
                static_cast<unsigned int>(std::lround(projection.theta)) %
                motion_table.getNumAngleBins();
            const double travel_m =
                static_cast<double>(motion_table.getTravelCost(
                    static_cast<unsigned int>(primitive_idx))) * xy_resolution_m;

            math::Pose2d successor_pose;
            successor_pose.x = static_cast<double>(projection.x) * xy_resolution_m;
            successor_pose.y = static_cast<double>(projection.y) * xy_resolution_m;
            successor_pose.theta = math::Angle::from_radians(
                motion_table.getAngleFromBin(successor_heading_bin));

            PlannerPrimitiveDebugEvent primitive_event;
            if (debug_trace != nullptr) {
                primitive_event.primitive_index = primitive_idx;
                primitive_event.turn_direction = turnDirectionName(projection.turn_dir);
                primitive_event.successor_pose = successor_pose;
                primitive_event.travel_m = travel_m;
            }

            const grid_map::Position successor_position(successor_pose.x, successor_pose.y);
            if (!costmap.isInside(successor_position)) {
                if (debug_trace != nullptr) {
                    primitive_event.outcome = "out_of_bounds";
                    primitive_event.detail = "successor_outside_costmap";
                    recordPrimitiveEvent(std::move(primitive_event));
                }
                return false;
            }

            if (current_profile.planner.max_cross_track_error_m > 0.0) {
                const double lane_distance = readRawLayerValue(
                    costmap, costs::CostmapLayerNames::LANE_DISTANCE, successor_position);
                if (std::isfinite(lane_distance) &&
                    lane_distance > current_profile.planner.max_cross_track_error_m) {
                    if (debug_trace != nullptr) {
                        primitive_event.outcome = "cross_track_pruned";
                        primitive_event.detail = "lane_distance_exceeded";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }
            }

            const ResolvedPlannerBehavior resolved = [&]() {
                common::ScopedProfilingTimer behavior_timer(
                    profiler, "planner.behavior_resolution");
                return PlannerBehaviorResolver::resolve(
                    successor_pose,
                    costmap,
                    current_node.zone,
                    current_node.behavior_name,
                    selection.selected_zones,
                    behavior_set_);
            }();
            if (resolved.profile == nullptr ||
                !isPrimitiveAllowed(projection.turn_dir, resolved)) {
                if (debug_trace != nullptr) {
                    primitive_event.resolved_behavior_name = resolved.behavior_name;
                    primitive_event.resolved_zone_name = zoneLabel(resolved.zone);
                    primitive_event.outcome =
                        resolved.profile == nullptr ? "behavior_unresolved" : "primitive_disallowed";
                    primitive_event.detail =
                        resolved.profile == nullptr ? "resolver_returned_null_profile"
                                                    : "primitive_not_allowed_for_behavior";
                    recordPrimitiveEvent(std::move(primitive_event));
                }
                return false;
            }
            if (debug_trace != nullptr) {
                primitive_event.resolved_behavior_name = resolved.behavior_name;
                primitive_event.resolved_zone_name = zoneLabel(resolved.zone);
            }

            const float successor_lethal_threshold = static_cast<float>(
                resolved.profile->collision_checker.lethal_threshold);
            if (std::abs(successor_lethal_threshold - collision_config.lethal_threshold) > 0.5f) {
                CollisionCheckerConfig adjusted_config = collision_config;
                adjusted_config.lethal_threshold = successor_lethal_threshold;
                CollisionChecker adjusted_checker(adjusted_config);
                const bool in_collision = [&]() {
                    common::ScopedProfilingTimer collision_timer(
                        profiler,
                        "planner.collision_check.primitive_adjusted_threshold");
                    return adjusted_checker.checkCollision(
                        costmap, car_, makeRobotState(successor_pose)).in_collision;
                }();
                if (in_collision) {
                    if (debug_trace != nullptr) {
                        primitive_event.outcome = "collision";
                        primitive_event.detail = "collision_with_adjusted_threshold";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }
            } else {
                const bool in_collision = [&]() {
                    common::ScopedProfilingTimer collision_timer(
                        profiler,
                        "planner.collision_check.primitive_default");
                    return collision_checker.checkCollision(
                        costmap, car_, makeRobotState(successor_pose)).in_collision;
                }();
                if (in_collision) {
                    if (debug_trace != nullptr) {
                        primitive_event.outcome = "collision";
                        primitive_event.detail = "collision_with_default_threshold";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }
            }

            const common::MotionDirection successor_motion =
                motionDirectionForPrimitive(projection.turn_dir);
            if (!canChangeMotionDirection(current_node, successor_motion)) {
                if (debug_trace != nullptr) {
                    primitive_event.outcome = "motion_change_blocked";
                    primitive_event.detail = "min_same_motion_length_not_satisfied";
                    recordPrimitiveEvent(std::move(primitive_event));
                }
                return false;
            }
            const double edge_cost = [&]() {
                common::ScopedProfilingTimer edge_cost_timer(
                    profiler, "planner.edge_cost");
                return computeEdgeCost(
                    costmap,
                    *resolved.profile,
                    current_node,
                    successor_position,
                    successor_pose.theta.radians(),
                    projection.turn_dir,
                    travel_m);
            }();
            const double new_g = current_node.g + edge_cost;

            const double successor_same_motion_length_m =
                computeNextSameMotionLength(
                    current_node,
                    successor_motion,
                    travel_m,
                    global_max_same_motion_length_m);
            const double successor_same_motion_remaining_to_change_m =
                computeNextSameMotionRemainingToChange(
                    current_node,
                    current_profile,
                    *resolved.profile,
                    successor_motion,
                    travel_m);

            const double holonomic_h =
                computeHolonomicHeuristic(costmap, successor_position, profiler);
            const double nh_h = computeNonHolonomicHeuristic(
                heuristic,
                selectHeuristicModel(resolved),
                successor_pose,
                request.goal,
                profiler);
            const double successor_h = std::max(holonomic_h, nh_h);

            SearchNode successor_node;
            successor_node.pose = successor_pose;
            successor_node.heading_bin = successor_heading_bin;
            successor_node.g = new_g;
            successor_node.h = successor_h;
            successor_node.parent_index = static_cast<int>(entry.node_index);
            successor_node.zone = resolved.zone;
            successor_node.behavior_name = resolved.behavior_name;
            successor_node.last_turn_class = classifyTurn(projection.turn_dir);
            successor_node.has_inbound_motion = true;
            successor_node.inbound_motion = motionDirectionForPrimitive(projection.turn_dir);
            successor_node.same_motion_length_m = successor_same_motion_length_m;
            successor_node.same_motion_remaining_to_change_m =
                successor_same_motion_remaining_to_change_m;

            const DiscreteStateKey successor_key = discretizeState(
                successor_pose,
                successor_heading_bin,
                successor_node,
                xy_resolution_m,
                global_max_same_motion_length_m);
            const auto existing = best_g_by_key.find(successor_key);
            if (existing != best_g_by_key.end() && new_g >= existing->second - 1e-6) {
                if (debug_trace != nullptr) {
                    primitive_event.edge_cost = edge_cost;
                    primitive_event.new_g = new_g;
                    primitive_event.successor_h = successor_h;
                    primitive_event.outcome = "dominated";
                    primitive_event.detail = "existing_state_has_better_or_equal_cost";
                    recordPrimitiveEvent(std::move(primitive_event));
                }
                return false;
            }

            const size_t successor_index = nodes.size();
            nodes.push_back(std::move(successor_node));
            best_g_by_key[successor_key] = new_g;
            open_queue.push(OpenEntry{
                new_g + successor_h,
                successor_h,
                insertion_order++,
                successor_index
            });
            debug_nodes_allocated = nodes.size();
            debug_unique_state_count = best_g_by_key.size();
            debug_open_queue_peak_size =
                std::max(debug_open_queue_peak_size, open_queue.size());
            if (debug_trace != nullptr) {
                primitive_event.edge_cost = edge_cost;
                primitive_event.new_g = new_g;
                primitive_event.successor_h = successor_h;
                primitive_event.outcome = "enqueued";
                primitive_event.detail = "successor_added_to_open_queue";
                recordPrimitiveEvent(std::move(primitive_event));
            }
            return true;
        };

        if (lane_following_node &&
            forward_primitive_idx < projections.size() &&
            tryExpandPrimitive(forward_primitive_idx)) {
            if (debug_trace != nullptr) {
                ++debug_trace->lane_suppression_forward_only_applied;
                expansion_debug.lane_suppression_forward_success = true;
                debug_trace->expansions.push_back(std::move(expansion_debug));
            }
            continue;
        }

        if (lane_following_node && forward_primitive_idx < projections.size()) {
            if (debug_trace != nullptr) {
                ++debug_trace->lane_suppression_fallbacks;
                expansion_debug.lane_suppression_fallback = true;
            }
        }

        for (size_t primitive_idx = 0; primitive_idx < projections.size(); ++primitive_idx) {
            tryExpandPrimitive(primitive_idx);
        }
        if (debug_trace != nullptr) {
            debug_trace->expansions.push_back(std::move(expansion_debug));
        }
    }

    if (debug_trace != nullptr) {
        debug_trace->terminal_reason =
            "Hybrid A* exhausted the search space without finding a path.";
    }
    return failWithDetail("Hybrid A* exhausted the search space without finding a path.");
}

HeuristicModel HybridAStarPlanner::selectHeuristicModel(
    const ResolvedPlannerBehavior& resolved_behavior) {
    if (allowsReverseMotion(resolved_behavior)) {
        return HeuristicModel::REEDS_SHEPP;
    }
    return HeuristicModel::DUBINS;
}

bool HybridAStarPlanner::isPrimitiveAllowed(
    TurnDirection turn_direction,
    const ResolvedPlannerBehavior& resolved_behavior) {
    if (!allowsReverseMotion(resolved_behavior)) {
        return turn_direction != TurnDirection::REVERSE &&
               turn_direction != TurnDirection::REV_LEFT &&
               turn_direction != TurnDirection::REV_RIGHT;
    }
    return true;
}

} // namespace planning
} // namespace coastmotionplanning
