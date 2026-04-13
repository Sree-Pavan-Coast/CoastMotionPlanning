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
    common::MotionDirection inbound_motion{common::MotionDirection::Forward};
};

struct DiscreteStateKey {
    int x_idx{0};
    int y_idx{0};
    int theta_idx{0};

    bool operator==(const DiscreteStateKey& other) const {
        return x_idx == other.x_idx &&
               y_idx == other.y_idx &&
               theta_idx == other.theta_idx;
    }
};

struct DiscreteStateKeyHash {
    size_t operator()(const DiscreteStateKey& key) const {
        size_t seed = std::hash<int>()(key.x_idx);
        seed ^= std::hash<int>()(key.y_idx) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.theta_idx) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
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
                                 double xy_resolution_m) {
    return DiscreteStateKey{
        static_cast<int>(std::lround(pose.x / xy_resolution_m)),
        static_cast<int>(std::lround(pose.y / xy_resolution_m)),
        static_cast<int>(heading_bin)
    };
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
    const bool reverse_motion =
        motionDirectionForPrimitive(turn_direction) == common::MotionDirection::Reverse;

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
    const auto deadline = Clock::now() + std::chrono::milliseconds(
        initial_profile.planner.max_planning_time_ms);

    const double normalized_start_yaw = normalizeAngleUnsigned(request.start.theta.radians());
    const unsigned int start_heading_bin =
        motion_table.getClosestAngularBin(normalized_start_yaw);
    const HeuristicModel start_model = start_zone->isReverseAllowed()
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
    nodes.push_back(start_node);

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open_queue;
    open_queue.push(OpenEntry{start_node.g + start_node.h, start_node.h, 0, 0});

    std::unordered_map<DiscreteStateKey, double, DiscreteStateKeyHash> best_g_by_key;
    best_g_by_key.emplace(discretizeState(request.start, start_heading_bin, xy_resolution_m), 0.0);

    uint64_t insertion_order = 1;

    while (!open_queue.empty()) {
        if (Clock::now() >= deadline) {
            failure_result.detail = "Hybrid A* search timed out before finding a path.";
            return failure_result;
        }

        const OpenEntry entry = open_queue.top();
        open_queue.pop();

        const SearchNode current_node = nodes[entry.node_index];
        const DiscreteStateKey current_key = discretizeState(
            current_node.pose, current_node.heading_bin, xy_resolution_m);
        const auto best_it = best_g_by_key.find(current_key);
        if (best_it == best_g_by_key.end() || current_node.g > best_it->second + 1e-6) {
            continue;
        }

        if (isGoalSatisfied(current_node,
                            request.goal,
                            goal_distance_tolerance,
                            yaw_bin_size_rad)) {
            return reconstructResult(nodes, static_cast<int>(entry.node_index));
        }

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
                current_node.zone,
                current_node.behavior_name,
                selection.selected_zones,
                behavior_set_);
            if (resolved.profile == nullptr ||
                !isPrimitiveAllowed(projection.turn_dir, resolved)) {
                continue;
            }

            CollisionCheckerConfig successor_collision_config = collision_config;
            successor_collision_config.lethal_threshold = static_cast<float>(
                resolved.profile->collision_checker.lethal_threshold);
            CollisionChecker successor_collision_checker(successor_collision_config);
            if (successor_collision_checker.checkCollision(
                    costmap, car_, makeRobotState(successor_pose)).in_collision) {
                continue;
            }

            const double travel_m =
                static_cast<double>(motion_table.getTravelCost(static_cast<unsigned int>(primitive_idx))) *
                xy_resolution_m;
            const double edge_cost = computeEdgeCost(
                costmap,
                *resolved.profile,
                current_node,
                successor_position,
                projection.turn_dir,
                travel_m);
            const double new_g = current_node.g + edge_cost;

            const DiscreteStateKey successor_key = discretizeState(
                successor_pose, successor_heading_bin, xy_resolution_m);
            const auto existing = best_g_by_key.find(successor_key);
            if (existing != best_g_by_key.end() && new_g >= existing->second - 1e-6) {
                continue;
            }

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
            successor_node.inbound_motion = motionDirectionForPrimitive(projection.turn_dir);

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
    return failure_result;
}

HeuristicModel HybridAStarPlanner::selectHeuristicModel(
    const ResolvedPlannerBehavior& resolved_behavior) {
    if (resolved_behavior.zone != nullptr && resolved_behavior.zone->isReverseAllowed()) {
        return HeuristicModel::REEDS_SHEPP;
    }
    return HeuristicModel::DUBINS;
}

bool HybridAStarPlanner::isPrimitiveAllowed(
    TurnDirection turn_direction,
    const ResolvedPlannerBehavior& resolved_behavior) {
    if (resolved_behavior.zone != nullptr && !resolved_behavior.zone->isReverseAllowed()) {
        return turn_direction != TurnDirection::REVERSE &&
               turn_direction != TurnDirection::REV_LEFT &&
               turn_direction != TurnDirection::REV_RIGHT;
    }
    return true;
}

} // namespace planning
} // namespace coastmotionplanning
