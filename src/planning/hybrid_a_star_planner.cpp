#include "coastmotionplanning/planning/hybrid_a_star_planner.hpp"

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

double computeHolonomicHeuristic(const grid_map::GridMap& costmap,
                                 const grid_map::Position& position) {
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
                                    const math::Pose2d& goal) {
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
        edge_cost += travel_m * (readLayerCost(costmap,
                                               costs::CostmapLayerNames::LANE_CENTERLINE_COST,
                                               successor_position) / CostValues::LETHAL);
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
                                           int goal_index) {
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
    HybridAStarPlannerResult failure_result;

    if (!behavior_set_.contains(request.initial_behavior_name)) {
        failure_result.detail =
            "Initial planner behavior '" + request.initial_behavior_name + "' is not defined.";
        return failure_result;
    }

    const PlannerBehaviorProfile& initial_profile =
        behavior_set_.get(request.initial_behavior_name);
    if (initial_profile.motion_primitives.min_turning_radius_m <= 0.0 ||
        initial_profile.motion_primitives.max_steer_angle_rad <= 0.0) {
        failure_result.detail =
            "Motion primitive constraints were not initialized from the selected robot model.";
        return failure_result;
    }

    ZoneSelectionResult selection;
    try {
        costs::ZoneSelector selector;
        selection = selector.select(
            request.start,
            request.goal,
            all_zones_,
            initial_profile.costmap.alpha_shape_alpha);
    } catch (const std::exception& e) {
        failure_result.detail = e.what();
        return failure_result;
    }

    const auto start_zone = costs::ZoneSelector::findContainingZone(
        geometry::Point2d(request.start.x, request.start.y), selection.selected_zones);
    if (start_zone == nullptr) {
        failure_result.detail = "Start pose is not inside the selected planning zones.";
        return failure_result;
    }

    costs::CostmapBuilder builder(
        initial_profile.makeCostmapConfig(), all_zones_, car_);
    grid_map::GridMap costmap = builder.build(selection, request.goal);

    CollisionCheckerConfig collision_config;
    collision_config.obstacle_layer = costs::CostmapLayerNames::COMBINED_COST;
    collision_config.lethal_threshold =
        static_cast<float>(initial_profile.collision_checker.lethal_threshold);
    CollisionChecker collision_checker(collision_config);

    if (collision_checker.checkCollision(costmap, car_, makeRobotState(request.start)).in_collision) {
        failure_result.detail = "Start pose is in collision on the selected-zone costmap.";
        return failure_result;
    }
    if (collision_checker.checkCollision(costmap, car_, makeRobotState(request.goal)).in_collision) {
        failure_result.detail = "Goal pose is in collision on the selected-zone costmap.";
        return failure_result;
    }

    MotionTableConfig motion_config;
    motion_config.minimum_turning_radius = static_cast<float>(
        initial_profile.motion_primitives.min_turning_radius_m /
        initial_profile.planner.xy_grid_resolution_m);
    motion_config.num_angle_quantization = static_cast<unsigned int>(
        initial_profile.motion_primitives.num_angle_bins);

    CarMotionTable motion_table;
    motion_table.initReedsShepp(motion_config);

    DualModelNonHolonomicHeuristic heuristic;
    heuristic.configureOmpl(
        static_cast<float>(initial_profile.motion_primitives.min_turning_radius_m));
    if (!request.dual_model_lut_path.empty()) {
        heuristic.loadFromFile(request.dual_model_lut_path);
    }

    const double xy_resolution_m = initial_profile.planner.xy_grid_resolution_m;
    const double yaw_bin_size_rad = motion_table.getBinSize();
    const double goal_distance_tolerance = initial_profile.planner.step_size_m;
    const double global_max_same_motion_length_m =
        computeGlobalMaxSameMotionLength(behavior_set_);
    const auto deadline = Clock::now() + std::chrono::milliseconds(
        initial_profile.planner.max_planning_time_ms);

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
        computeHolonomicHeuristic(costmap, grid_map::Position(request.start.x, request.start.y)),
        computeNonHolonomicHeuristic(heuristic, start_model, request.start, request.goal));

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

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open_queue;
    open_queue.push(OpenEntry{start_node.g + start_node.h, start_node.h, 0, 0});

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

    uint64_t insertion_order = 1;
    uint64_t expansion_count = 0;

    // Lambda: attempt analytic expansion from a node to the goal.
    // Returns a result with success=true if a collision-free path is found.
    auto tryAnalyticExpansion = [&](const SearchNode& node,
                                   size_t node_index) -> HybridAStarPlannerResult {
        HybridAStarPlannerResult empty;
        if (!heuristic.hasOmplSpaces()) {
            return empty;
        }

        // Don't expand if the node hasn't satisfied its minimum same-motion run
        if (node.same_motion_remaining_to_change_m > MOTION_EPSILON) {
            return empty;
        }

        const PlannerBehaviorProfile& node_profile =
            behavior_set_.get(node.behavior_name);
        const HeuristicModel model =
            allowsReverseMotion(node_profile, node.zone)
            ? HeuristicModel::REEDS_SHEPP
            : HeuristicModel::DUBINS;

        auto [path_length, waypoints] = heuristic.samplePath(
            model,
            node.pose.x, node.pose.y, node.pose.theta.radians(),
            request.goal.x, request.goal.y, request.goal.theta.radians(),
            analytic_step_size);

        if (waypoints.empty() || path_length > analytic_max_length) {
            return empty;
        }

        // Enforce min_path_len_in_same_motion: the terminal forward segment
        // must be long enough for the robot to stop.
        const double effective_terminal_length =
            (node.has_inbound_motion &&
             node.inbound_motion == common::MotionDirection::Forward)
            ? node.same_motion_length_m + path_length
            : path_length;
        if (effective_terminal_length <
            node_profile.planner.min_path_len_in_same_motion - MOTION_EPSILON) {
            return empty;
        }

        // Check all waypoints for collision and resolve behaviors
        std::vector<std::string> wp_behaviors;
        wp_behaviors.reserve(waypoints.size());
        for (const auto& wp : waypoints) {
            robot::RobotState rs;
            rs.x = wp[0];
            rs.y = wp[1];
            rs.yaw = wp[2];

            const grid_map::Position wp_pos(rs.x, rs.y);
            if (!costmap.isInside(wp_pos)) {
                return empty;
            }

            if (collision_checker.checkCollision(costmap, car_, rs).in_collision) {
                return empty;
            }

            // Resolve zone/behavior for this waypoint
            math::Pose2d wp_pose(wp[0], wp[1],
                                 math::Angle::from_radians(wp[2]));
            const ResolvedPlannerBehavior resolved = PlannerBehaviorResolver::resolve(
                wp_pose, costmap, node.zone, node.behavior_name,
                selection.selected_zones, behavior_set_);
            wp_behaviors.push_back(
                resolved.profile != nullptr
                    ? resolved.behavior_name
                    : node.behavior_name);
        }

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
            prefix_behaviors.push_back(wp_behaviors[i]);
            prefix_directions.push_back(common::MotionDirection::Forward);
        }

        result.poses = std::move(prefix_poses);
        result.behavior_sequence = std::move(prefix_behaviors);
        result.segment_directions = std::move(prefix_directions);
        result.detail = "Path found (analytic expansion).";
        return result;
    };

    while (!open_queue.empty()) {
        if (Clock::now() >= deadline) {
            failure_result.detail = "Hybrid A* search timed out before finding a path.";
            return failure_result;
        }

        const OpenEntry entry = open_queue.top();
        open_queue.pop();

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
            continue;
        }

        if (isGoalSatisfied(current_node,
                            request.goal,
                            goal_distance_tolerance,
                            yaw_bin_size_rad) &&
            isTerminalMotionSegmentValid(current_node)) {
            return reconstructResult(nodes, static_cast<int>(entry.node_index));
        }

        // Analytic expansion: periodically try a direct curve to the goal
        ++expansion_count;
        if (expansion_count % analytic_expansion_interval == 0) {
            auto analytic_result = tryAnalyticExpansion(current_node, entry.node_index);
            if (analytic_result.success) {
                return analytic_result;
            }
        }

        const PlannerBehaviorProfile& current_profile =
            behavior_set_.get(current_node.behavior_name);

        const auto projections = motion_table.getProjections(
            static_cast<float>(current_node.pose.x / xy_resolution_m),
            static_cast<float>(current_node.pose.y / xy_resolution_m),
            current_node.heading_bin);

        for (size_t primitive_idx = 0; primitive_idx < projections.size(); ++primitive_idx) {
            const auto& projection = projections[primitive_idx];
            const unsigned int successor_heading_bin =
                static_cast<unsigned int>(std::lround(projection.theta)) %
                motion_table.getNumAngleBins();

            math::Pose2d successor_pose;
            successor_pose.x = static_cast<double>(projection.x) * xy_resolution_m;
            successor_pose.y = static_cast<double>(projection.y) * xy_resolution_m;
            successor_pose.theta = math::Angle::from_radians(
                motion_table.getAngleFromBin(successor_heading_bin));

            const grid_map::Position successor_position(successor_pose.x, successor_pose.y);
            if (!costmap.isInside(successor_position)) {
                continue;
            }

            const ResolvedPlannerBehavior resolved = PlannerBehaviorResolver::resolve(
                successor_pose,
                costmap,
                current_node.zone,
                current_node.behavior_name,
                selection.selected_zones,
                behavior_set_);
            if (resolved.profile == nullptr ||
                !isPrimitiveAllowed(projection.turn_dir, resolved)) {
                continue;
            }

            const float successor_lethal_threshold = static_cast<float>(
                resolved.profile->collision_checker.lethal_threshold);
            if (std::abs(successor_lethal_threshold - collision_config.lethal_threshold) > 0.5f) {
                CollisionCheckerConfig adjusted_config = collision_config;
                adjusted_config.lethal_threshold = successor_lethal_threshold;
                CollisionChecker adjusted_checker(adjusted_config);
                if (adjusted_checker.checkCollision(
                        costmap, car_, makeRobotState(successor_pose)).in_collision) {
                    continue;
                }
            } else {
                if (collision_checker.checkCollision(
                        costmap, car_, makeRobotState(successor_pose)).in_collision) {
                    continue;
                }
            }

            const double travel_m =
                static_cast<double>(motion_table.getTravelCost(static_cast<unsigned int>(primitive_idx))) *
                xy_resolution_m;
            const common::MotionDirection successor_motion =
                motionDirectionForPrimitive(projection.turn_dir);
            if (!canChangeMotionDirection(current_node, successor_motion)) {
                continue;
            }
            const double edge_cost = computeEdgeCost(
                costmap,
                *resolved.profile,
                current_node,
                successor_position,
                projection.turn_dir,
                travel_m);
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
                computeHolonomicHeuristic(costmap, successor_position);
            const double nh_h = computeNonHolonomicHeuristic(
                heuristic,
                selectHeuristicModel(resolved),
                successor_pose,
                request.goal);
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
                continue;
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
        }
    }

    failure_result.detail = "Hybrid A* exhausted the search space without finding a path.";
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
