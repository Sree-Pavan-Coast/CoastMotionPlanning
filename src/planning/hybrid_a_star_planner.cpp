#include "coastmotionplanning/planning/hybrid_a_star_planner.hpp"

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
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
constexpr double GOAL_APPROACH_TURNING_PENALTY_PER_M = 5.0;

enum class TurnClass {
    UNKNOWN = 0,
    STRAIGHT,
    LEFT,
    RIGHT
};

enum class FrontierHeuristicMode {
    FinalGoal = 0,
    StageFrontier
};

struct SearchNode {
    math::Pose2d pose;
    size_t frontier_id{0};
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
    FrontierHeuristicMode heuristic_mode{FrontierHeuristicMode::FinalGoal};
    double stage_heuristic_value{std::numeric_limits<double>::quiet_NaN()};
    double final_goal_holonomic_value{std::numeric_limits<double>::quiet_NaN()};
    double nonholonomic_heuristic_value{std::numeric_limits<double>::quiet_NaN()};
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
    double g{0.0};
    uint64_t insertion_order{0};
    size_t node_index{0};
    DiscreteStateKey state_key{};

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
    std::string detail;
    std::vector<common::MotionDirection> segment_directions;
};

struct SearchContext {
    std::string behavior_name;
    const PlannerBehaviorProfile* profile{nullptr};
    CarMotionTable motion_table;
    double xy_resolution_m{0.0};
    double yaw_bin_size_rad{0.0};
    double goal_distance_tolerance{0.0};
    double analytic_max_length_m{0.0};
    unsigned int analytic_expansion_interval{1};
    double analytic_step_size{0.0};
};

struct FrontierCounters {
    uint64_t expansions_popped{0};
    uint64_t stale_entries_skipped{0};
    uint64_t goal_checks{0};
    uint64_t goal_hits{0};
    uint64_t lane_following_candidates{0};
    uint64_t lane_suppression_forward_only_applied{0};
    uint64_t lane_suppression_fallbacks{0};
    uint64_t analytic_attempts{0};
    uint64_t analytic_successes{0};
    uint64_t analytic_fail_no_ompl{0};
    uint64_t analytic_fail_same_motion_guard{0};
    uint64_t analytic_fail_path_length{0};
    uint64_t analytic_fail_out_of_bounds{0};
    uint64_t analytic_fail_collision{0};
    uint64_t analytic_fail_validation{0};
    uint64_t primitive_out_of_bounds{0};
    uint64_t primitive_cross_track_pruned{0};
    uint64_t primitive_behavior_unresolved{0};
    uint64_t primitive_disallowed{0};
    uint64_t primitive_collision{0};
    uint64_t primitive_motion_change_blocked{0};
    uint64_t primitive_dominated{0};
    uint64_t primitive_enqueued{0};
};

struct FrontierRuntime {
    costs::SearchFrontierDescriptor descriptor;
    const SearchContext* context{nullptr};
    std::string stage_heuristic_layer_name;
    mutable std::mutex mutex;
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open_queue;
    std::unordered_map<DiscreteStateKey, double, DiscreteStateKeyHash> best_g_by_key;
    common::ProfilingCollector profiling_collector;
    FrontierCounters counters;
    PlannerFrontierDebugSummary debug_summary;
    std::vector<PlannerExpansionDebugEvent> expansion_events;
};

struct SharedPlannerResult {
    HybridAStarPlannerResult result;
    bool published{false};
};

struct FrontierHeuristicEvaluation {
    FrontierHeuristicMode mode{FrontierHeuristicMode::FinalGoal};
    double combined_h{LARGE_COST};
    double stage_heuristic_value{std::numeric_limits<double>::quiet_NaN()};
    double final_goal_holonomic_value{std::numeric_limits<double>::quiet_NaN()};
    double nonholonomic_heuristic_value{std::numeric_limits<double>::quiet_NaN()};
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

std::string frontierRoleName(costs::SearchFrontierRole role) {
    switch (role) {
    case costs::SearchFrontierRole::StartZone:
        return "start_zone";
    case costs::SearchFrontierRole::Transition:
        return "transition";
    case costs::SearchFrontierRole::GoalZone:
        return "goal_zone";
    }
    return "frontier";
}

std::string frontierHeuristicModeName(FrontierHeuristicMode mode) {
    switch (mode) {
    case FrontierHeuristicMode::StageFrontier:
        return "stage_frontier";
    case FrontierHeuristicMode::FinalGoal:
        return "final_goal";
    }
    return "final_goal";
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

double readHolonomicLayerValue(const grid_map::GridMap& costmap,
                               const std::string& layer_name,
                               const grid_map::Position& position,
                               common::ProfilingCollector* profiler = nullptr) {
    common::ScopedProfilingTimer timer(profiler, "planner.holonomic_heuristic_read");
    return readRawLayerValue(costmap, layer_name, position);
}

double computeHolonomicLayerHeuristic(const grid_map::GridMap& costmap,
                                      const std::string& layer_name,
                                      const grid_map::Position& position,
                                      common::ProfilingCollector* profiler = nullptr) {
    const double value = readHolonomicLayerValue(costmap, layer_name, position, profiler);
    if (std::isnan(value)) {
        return LARGE_COST;
    }
    return value;
}

double computeHolonomicHeuristic(const grid_map::GridMap& costmap,
                                 const grid_map::Position& position,
                                 common::ProfilingCollector* profiler = nullptr) {
    return computeHolonomicLayerHeuristic(
        costmap,
        costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES,
        position,
        profiler);
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

FrontierHeuristicEvaluation computeFrontierHeuristic(
    const grid_map::GridMap& costmap,
    const FrontierRuntime& frontier,
    const DualModelNonHolonomicHeuristic& heuristic,
    HeuristicModel final_goal_model,
    const math::Pose2d& from,
    const math::Pose2d& goal,
    common::ProfilingCollector* profiler = nullptr) {
    FrontierHeuristicEvaluation result;
    const grid_map::Position position(from.x, from.y);

    if (!frontier.stage_heuristic_layer_name.empty()) {
        result.stage_heuristic_value = readHolonomicLayerValue(
            costmap,
            frontier.stage_heuristic_layer_name,
            position,
            profiler);
        if (!std::isnan(result.stage_heuristic_value)) {
            result.mode = FrontierHeuristicMode::StageFrontier;
            result.combined_h = result.stage_heuristic_value;
            return result;
        }
    }

    result.final_goal_holonomic_value =
        computeHolonomicHeuristic(costmap, position, profiler);
    result.nonholonomic_heuristic_value = computeNonHolonomicHeuristic(
        heuristic,
        final_goal_model,
        from,
        goal,
        profiler);
    result.combined_h = std::max(
        result.final_goal_holonomic_value,
        result.nonholonomic_heuristic_value);
    return result;
}

double computeGoalApproachStraightPenalty(const PlannerBehaviorProfile& profile,
                                          const math::Pose2d& successor_pose,
                                          const math::Pose2d& goal,
                                          TurnDirection turn_direction,
                                          double travel_m) {
    if (profile.planner.goal_approach_straight_distance_m <= MOTION_EPSILON) {
        return 0.0;
    }

    const TurnClass turn_class = classifyTurn(turn_direction);
    if (turn_class != TurnClass::LEFT && turn_class != TurnClass::RIGHT) {
        return 0.0;
    }

    const double distance_to_goal = std::hypot(
        goal.x - successor_pose.x,
        goal.y - successor_pose.y);
    if (distance_to_goal > profile.planner.goal_approach_straight_distance_m) {
        return 0.0;
    }

    return GOAL_APPROACH_TURNING_PENALTY_PER_M * travel_m;
}

bool analyticGoalApproachIsStraightEnough(
    const std::vector<std::array<double, 3>>& waypoints,
    double goal_approach_straight_distance_m,
    double yaw_bin_size_rad) {
    if (goal_approach_straight_distance_m <= MOTION_EPSILON) {
        return true;
    }

    double remaining_suffix_m = goal_approach_straight_distance_m;
    const double goal_heading_rad = waypoints.back()[2];
    for (size_t i = waypoints.size() - 1; i > 0 && remaining_suffix_m > MOTION_EPSILON; --i) {
        const auto& from = waypoints[i - 1];
        const auto& to = waypoints[i];
        const double segment_length_m = std::hypot(to[0] - from[0], to[1] - from[1]);
        if (segment_length_m <= MOTION_EPSILON) {
            continue;
        }

        const double from_goal_heading_delta =
            std::abs(normalizeAngleSigned(from[2] - goal_heading_rad));
        const double to_goal_heading_delta =
            std::abs(normalizeAngleSigned(to[2] - goal_heading_rad));
        const double heading_delta =
            std::abs(normalizeAngleSigned(to[2] - from[2]));
        if (from_goal_heading_delta > yaw_bin_size_rad ||
            to_goal_heading_delta > yaw_bin_size_rad ||
            heading_delta > yaw_bin_size_rad) {
            return false;
        }

        remaining_suffix_m -= segment_length_m;
    }

    return true;
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
    double global_max_same_motion_length_m,
    double yaw_bin_size_rad) {
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
        return result;
    }

    const PlannerBehaviorProfile& goal_profile = *waypoint_resolutions.back().profile;
    if (!analyticGoalApproachIsStraightEnough(
            waypoints,
            goal_profile.planner.goal_approach_straight_distance_m,
            yaw_bin_size_rad)) {
        result.valid = false;
        result.detail = "goal_approach_turning_suffix";
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

HybridAStarPlannerResult reconstructResult(const std::deque<SearchNode>& nodes,
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

SearchContext buildSearchContext(const std::string& behavior_name,
                                 const PlannerBehaviorSet& behavior_set) {
    SearchContext context;
    context.behavior_name = behavior_name;
    context.profile = &behavior_set.get(behavior_name);
    context.xy_resolution_m = context.profile->planner.xy_grid_resolution_m;

    MotionTableConfig motion_config;
    motion_config.minimum_turning_radius = static_cast<float>(
        context.profile->motion_primitives.min_turning_radius_m /
        context.profile->planner.xy_grid_resolution_m);
    motion_config.num_angle_quantization = static_cast<unsigned int>(
        context.profile->motion_primitives.num_angle_bins);
    context.motion_table.initReedsShepp(motion_config);
    context.yaw_bin_size_rad = context.motion_table.getBinSize();
    context.goal_distance_tolerance = context.profile->planner.step_size_m;
    context.analytic_max_length_m =
        context.profile->planner.analytic_expansion_max_length_m;
    const double analytic_ratio = context.profile->planner.analytic_expansion_ratio;
    context.analytic_expansion_interval = std::max(
        1u, static_cast<unsigned int>(
                std::round(1.0 / std::max(analytic_ratio, 0.01))));
    context.analytic_step_size = context.xy_resolution_m * 2.0;
    return context;
}

void mergeFrontierCountersIntoTrace(const FrontierCounters& counters,
                                    HybridAStarPlannerDebugTrace* trace) {
    if (trace == nullptr) {
        return;
    }

    trace->expansions_popped += counters.expansions_popped;
    trace->stale_entries_skipped += counters.stale_entries_skipped;
    trace->goal_checks += counters.goal_checks;
    trace->goal_hits += counters.goal_hits;
    trace->lane_following_candidates += counters.lane_following_candidates;
    trace->lane_suppression_forward_only_applied +=
        counters.lane_suppression_forward_only_applied;
    trace->lane_suppression_fallbacks += counters.lane_suppression_fallbacks;
    trace->analytic_attempts += counters.analytic_attempts;
    trace->analytic_successes += counters.analytic_successes;
    trace->analytic_fail_no_ompl += counters.analytic_fail_no_ompl;
    trace->analytic_fail_same_motion_guard += counters.analytic_fail_same_motion_guard;
    trace->analytic_fail_path_length += counters.analytic_fail_path_length;
    trace->analytic_fail_out_of_bounds += counters.analytic_fail_out_of_bounds;
    trace->analytic_fail_collision += counters.analytic_fail_collision;
    trace->analytic_fail_validation += counters.analytic_fail_validation;
    trace->primitive_out_of_bounds += counters.primitive_out_of_bounds;
    trace->primitive_cross_track_pruned += counters.primitive_cross_track_pruned;
    trace->primitive_behavior_unresolved += counters.primitive_behavior_unresolved;
    trace->primitive_disallowed += counters.primitive_disallowed;
    trace->primitive_collision += counters.primitive_collision;
    trace->primitive_motion_change_blocked += counters.primitive_motion_change_blocked;
    trace->primitive_dominated += counters.primitive_dominated;
    trace->primitive_enqueued += counters.primitive_enqueued;
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
    common::ProfilingCollector attempt_profiling_collector;
    common::ProfilingCollector* profiler =
        debug_trace != nullptr ? &attempt_profiling_collector : nullptr;
    Clock::time_point search_loop_start_time{};
    bool search_loop_started = false;
    std::vector<std::unique_ptr<FrontierRuntime>> frontier_runtimes;
    std::unordered_map<uint64_t, PlannerFrontierHandoffDebugSummary> handoff_summaries;

    if (debug_trace != nullptr) {
        debug_trace->initial_behavior_name = request.initial_behavior_name;
        debug_trace->transition_behavior_name = request.transition_behavior_name;
    }

    const auto finalizeResult =
        [&](HybridAStarPlannerResult result,
            const std::deque<SearchNode>& nodes,
            size_t queue_peak_size) -> HybridAStarPlannerResult {
        if (debug_trace != nullptr) {
            if (search_loop_started) {
                debug_trace->search_loop_ms =
                    elapsedMilliseconds(search_loop_start_time, Clock::now());
            }
            debug_trace->nodes_allocated = nodes.size();
            debug_trace->unique_state_count = 0;
            debug_trace->open_queue_peak_size = queue_peak_size;
            for (const auto& runtime_ptr : frontier_runtimes) {
                auto& runtime = *runtime_ptr;
                {
                    std::lock_guard<std::mutex> lock(runtime.mutex);
                    runtime.debug_summary.closed_set_size = runtime.best_g_by_key.size();
                    runtime.debug_summary.profiling_scopes =
                        runtime.profiling_collector.snapshotSortedByTotalMs();
                    debug_trace->unique_state_count += runtime.best_g_by_key.size();
                }
                mergeFrontierCountersIntoTrace(runtime.counters, debug_trace.get());
                debug_trace->frontier_summaries.push_back(runtime.debug_summary);
                debug_trace->expansions.insert(
                    debug_trace->expansions.end(),
                    runtime.expansion_events.begin(),
                    runtime.expansion_events.end());
                attempt_profiling_collector.mergeFrom(runtime.profiling_collector);
            }
            std::sort(
                debug_trace->expansions.begin(),
                debug_trace->expansions.end(),
                [](const PlannerExpansionDebugEvent& lhs,
                   const PlannerExpansionDebugEvent& rhs) {
                    return lhs.expansion_index < rhs.expansion_index;
                });
            for (const auto& entry : handoff_summaries) {
                debug_trace->frontier_handoffs.push_back(entry.second);
            }
            std::sort(
                debug_trace->frontier_handoffs.begin(),
                debug_trace->frontier_handoffs.end(),
                [](const PlannerFrontierHandoffDebugSummary& lhs,
                   const PlannerFrontierHandoffDebugSummary& rhs) {
                    if (lhs.from_frontier_id != rhs.from_frontier_id) {
                        return lhs.from_frontier_id < rhs.from_frontier_id;
                    }
                    return lhs.to_frontier_id < rhs.to_frontier_id;
                });
            debug_trace->total_planning_ms =
                elapsedMilliseconds(plan_start_time, Clock::now());
            debug_trace->profiling_scopes =
                attempt_profiling_collector.snapshotSortedByTotalMs();
            result.debug_trace = debug_trace;
        }
        return result;
    };

    const auto failWithDetail =
        [&](std::string detail,
            const std::deque<SearchNode>& nodes = {}) -> HybridAStarPlannerResult {
        failure_result.detail = std::move(detail);
        if (debug_trace != nullptr) {
            debug_trace->terminal_reason = failure_result.detail;
        }
        return finalizeResult(failure_result, nodes, 0);
    };

    if (!behavior_set_.contains(request.initial_behavior_name)) {
        return failWithDetail(
            "Initial planner behavior '" + request.initial_behavior_name + "' is not defined.");
    }
    if (!request.transition_behavior_name.empty() &&
        !behavior_set_.contains(request.transition_behavior_name)) {
        return failWithDetail(
            "Transition planner behavior '" + request.transition_behavior_name +
            "' is not defined.");
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
            initial_profile.costmap.alpha_shape_alpha,
            request.initial_behavior_name,
            request.transition_behavior_name);
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
        for (const auto& stage_layer : builder.getStageHeuristicLayerSummaries()) {
            debug_trace->stage_heuristic_layers.push_back(PlannerStageHeuristicDebugSummary{
                stage_layer.source_frontier_id,
                stage_layer.target_frontier_id,
                stage_layer.layer_name,
                stage_layer.seed_cell_count,
                stage_layer.finite_cell_count,
                stage_layer.min_finite_value,
                stage_layer.max_finite_value
            });
        }
    }
    std::unordered_map<size_t, std::string> stage_layer_name_by_frontier;
    for (const auto& stage_layer : builder.getStageHeuristicLayerSummaries()) {
        stage_layer_name_by_frontier.emplace(
            stage_layer.source_frontier_id,
            stage_layer.layer_name);
    }

    CollisionCheckerConfig default_collision_config;
    default_collision_config.obstacle_layer = costs::CostmapLayerNames::COMBINED_COST;
    default_collision_config.lethal_threshold =
        static_cast<float>(initial_profile.collision_checker.lethal_threshold);
    CollisionChecker default_collision_checker(default_collision_config);

    const bool start_in_collision = [&]() {
        common::ScopedProfilingTimer timer(
            profiler, "planner.collision_check.start_validation");
        return default_collision_checker.checkCollision(
            costmap, car_, makeRobotState(request.start)).in_collision;
    }();
    if (start_in_collision) {
        return failWithDetail("Start pose is in collision on the selected-zone costmap.");
    }
    const bool goal_in_collision = [&]() {
        common::ScopedProfilingTimer timer(
            profiler, "planner.collision_check.goal_validation");
        return default_collision_checker.checkCollision(
            costmap, car_, makeRobotState(request.goal)).in_collision;
    }();
    if (goal_in_collision) {
        return failWithDetail("Goal pose is in collision on the selected-zone costmap.");
    }

    const Clock::time_point heuristic_setup_start_time = Clock::now();
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

    std::unordered_map<std::string, SearchContext> contexts_by_behavior;
    for (const auto& frontier : selection.frontiers) {
        const std::string behavior_name =
            frontier.behavior_name.empty()
                ? request.initial_behavior_name
                : frontier.behavior_name;
        if (contexts_by_behavior.find(behavior_name) == contexts_by_behavior.end()) {
            contexts_by_behavior.emplace(
                behavior_name,
                buildSearchContext(behavior_name, behavior_set_));
        }
    }

    frontier_runtimes.reserve(selection.frontiers.size());
    for (const auto& frontier : selection.frontiers) {
        auto runtime = std::make_unique<FrontierRuntime>();
        runtime->descriptor = frontier;
        if (runtime->descriptor.behavior_name.empty()) {
            runtime->descriptor.behavior_name = request.initial_behavior_name;
        }
        runtime->context =
            &contexts_by_behavior.at(runtime->descriptor.behavior_name);
        const auto stage_layer_it =
            stage_layer_name_by_frontier.find(runtime->descriptor.frontier_id);
        if (stage_layer_it != stage_layer_name_by_frontier.end()) {
            runtime->stage_heuristic_layer_name = stage_layer_it->second;
        }
        runtime->best_g_by_key.reserve(50000);
        runtime->debug_summary.frontier_id = runtime->descriptor.frontier_id;
        runtime->debug_summary.frontier_role =
            frontierRoleName(runtime->descriptor.role);
        runtime->debug_summary.zone_name = zoneLabel(runtime->descriptor.zone);
        runtime->debug_summary.behavior_name = runtime->descriptor.behavior_name;
        frontier_runtimes.push_back(std::move(runtime));
    }

    const double global_max_same_motion_length_m =
        computeGlobalMaxSameMotionLength(behavior_set_);
    const auto deadline = Clock::now() + std::chrono::milliseconds(
        initial_profile.planner.max_planning_time_ms);
    search_loop_start_time = Clock::now();
    search_loop_started = true;

    const SearchContext& start_context = *frontier_runtimes.front()->context;
    const double normalized_start_yaw = normalizeAngleUnsigned(request.start.theta.radians());
    const unsigned int start_heading_bin =
        start_context.motion_table.getClosestAngularBin(normalized_start_yaw);
    const HeuristicModel start_model = allowsReverseMotion(*start_context.profile, start_zone)
        ? HeuristicModel::REEDS_SHEPP
        : HeuristicModel::DUBINS;
    const FrontierHeuristicEvaluation start_heuristic = computeFrontierHeuristic(
        costmap,
        *frontier_runtimes.front(),
        heuristic,
        start_model,
        request.start,
        request.goal,
        profiler);

    std::deque<SearchNode> nodes;
    nodes.push_back(SearchNode{
        request.start,
        0,
        start_heading_bin,
        0.0,
        start_heuristic.combined_h,
        -1,
        start_zone,
        frontier_runtimes.front()->descriptor.behavior_name,
        TurnClass::UNKNOWN,
        false,
        common::MotionDirection::Forward,
        0.0,
        0.0,
        start_heuristic.mode,
        start_heuristic.stage_heuristic_value,
        start_heuristic.final_goal_holonomic_value,
        start_heuristic.nonholonomic_heuristic_value
    });

    const DiscreteStateKey start_key = discretizeState(
        request.start,
        start_heading_bin,
        nodes.front(),
        start_context.xy_resolution_m,
        global_max_same_motion_length_m);
    {
        auto& start_frontier = *frontier_runtimes.front();
        std::lock_guard<std::mutex> lock(start_frontier.mutex);
        start_frontier.best_g_by_key.emplace(start_key, 0.0);
        start_frontier.open_queue.push(OpenEntry{
            nodes.front().g + nodes.front().h,
            nodes.front().h,
            nodes.front().g,
            0,
            0,
            start_key
        });
        start_frontier.debug_summary.enqueued = 1;
        start_frontier.debug_summary.open_queue_peak_size = 1;
        start_frontier.debug_summary.first_enqueue_ms = 0.0;
    }

    std::mutex nodes_mutex;
    std::mutex result_mutex;
    std::mutex handoff_mutex;
    std::mutex scheduler_mutex;
    std::condition_variable scheduler_cv;
    SharedPlannerResult shared_result;
    std::string success_terminal_reason;

    std::atomic<bool> success_found{false};
    std::atomic<bool> shutdown{false};
    std::atomic<bool> deadline_hit{false};
    std::atomic<size_t> total_queue_entries{1};
    std::atomic<size_t> active_workers{0};
    std::atomic<size_t> total_queue_peak_size{1};
    std::atomic<uint64_t> insertion_order{1};
    std::atomic<size_t> global_expansion_index{0};

    const auto maybeSignalQuiescence = [&]() {
        if (!success_found.load() &&
            total_queue_entries.load() == 0 &&
            active_workers.load() == 0) {
            shutdown.store(true);
            scheduler_cv.notify_all();
        }
    };

    const auto recordHandoff = [&](size_t from_frontier_id, size_t to_frontier_id) {
        if (from_frontier_id == to_frontier_id) {
            return;
        }
        const uint64_t key =
            (static_cast<uint64_t>(from_frontier_id) << 32) |
            static_cast<uint64_t>(to_frontier_id);
        std::lock_guard<std::mutex> lock(handoff_mutex);
        auto& summary = handoff_summaries[key];
        summary.from_frontier_id = from_frontier_id;
        summary.to_frontier_id = to_frontier_id;
        ++summary.transfer_count;
        if (summary.first_transfer_ms < 0.0) {
            summary.first_transfer_ms =
                elapsedMilliseconds(search_loop_start_time, Clock::now());
        }
    };

    const auto enqueueSuccessor =
        [&](size_t destination_frontier_id,
            SearchNode successor_node,
            const DiscreteStateKey& successor_key,
            double successor_h,
            double new_g) -> std::optional<size_t> {
        auto& frontier = *frontier_runtimes[destination_frontier_id];
        std::lock_guard<std::mutex> frontier_lock(frontier.mutex);
        const auto existing = frontier.best_g_by_key.find(successor_key);
        if (existing != frontier.best_g_by_key.end() &&
            new_g >= existing->second - 1e-6) {
            return std::nullopt;
        }

        size_t successor_index = 0;
        {
            std::lock_guard<std::mutex> nodes_lock(nodes_mutex);
            successor_index = nodes.size();
            nodes.push_back(std::move(successor_node));
        }

        frontier.best_g_by_key[successor_key] = new_g;
        frontier.open_queue.push(OpenEntry{
            new_g + successor_h,
            successor_h,
            new_g,
            insertion_order.fetch_add(1),
            successor_index,
            successor_key
        });
        ++frontier.debug_summary.enqueued;
        if (frontier.debug_summary.first_enqueue_ms < 0.0) {
            frontier.debug_summary.first_enqueue_ms =
                elapsedMilliseconds(search_loop_start_time, Clock::now());
        }
        frontier.debug_summary.open_queue_peak_size = std::max(
            frontier.debug_summary.open_queue_peak_size,
            frontier.open_queue.size());
        total_queue_entries.fetch_add(1);
        total_queue_peak_size.store(std::max(
            total_queue_peak_size.load(),
            total_queue_entries.load()));
        scheduler_cv.notify_all();
        return successor_index;
    };

    struct AnalyticAttemptOutcome {
        HybridAStarPlannerResult result;
        PlannerAnalyticDebugEvent debug_event;
    };

    const auto worker = [&](size_t frontier_index) {
        FrontierRuntime& frontier = *frontier_runtimes[frontier_index];
        uint64_t local_expansion_count = 0;

        const auto tryAnalyticExpansion =
            [&](const SearchNode& node,
                size_t node_index,
                const std::string& trigger) -> AnalyticAttemptOutcome {
            common::ScopedProfilingTimer analytic_timer(
                &frontier.profiling_collector, "planner.analytic_expansion_attempt");
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
            if (node.same_motion_remaining_to_change_m > MOTION_EPSILON) {
                outcome.debug_event.outcome = "failed";
                outcome.debug_event.detail = "same_motion_guard";
                return outcome;
            }

            const FrontierRuntime& node_frontier = *frontier_runtimes[node.frontier_id];
            const SearchContext& node_context = *node_frontier.context;
            const PlannerBehaviorProfile& node_profile =
                *node_context.profile;
            const HeuristicModel model =
                allowsReverseMotion(node_profile, node.zone)
                    ? HeuristicModel::REEDS_SHEPP
                    : HeuristicModel::DUBINS;

            std::pair<double, std::vector<std::array<double, 3>>> sampled_path;
            {
                common::ScopedProfilingTimer sample_path_timer(
                    &frontier.profiling_collector, "planner.analytic_sample_path");
                sampled_path = heuristic.samplePath(
                    model,
                    node.pose.x, node.pose.y, node.pose.theta.radians(),
                    request.goal.x, request.goal.y, request.goal.theta.radians(),
                    node_context.analytic_step_size);
            }
            auto& path_length = sampled_path.first;
            auto& waypoints = sampled_path.second;
            outcome.debug_event.path_length_m = path_length;
            outcome.debug_event.waypoint_count = waypoints.size();

            if (waypoints.empty() || path_length > node_context.analytic_max_length_m) {
                outcome.debug_event.outcome = "failed";
                outcome.debug_event.detail =
                    waypoints.empty() ? "no_waypoints" : "path_length_exceeded";
                return outcome;
            }

            std::vector<ResolvedPlannerBehavior> wp_resolutions;
            wp_resolutions.reserve(waypoints.size());
            wp_resolutions.push_back(ResolvedPlannerBehavior{
                node.frontier_id,
                node.zone,
                node.behavior_name,
                &node_profile,
                false,
                false
            });
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
                        &frontier.profiling_collector, "planner.collision_check.analytic");
                    return default_collision_checker.checkCollision(
                        costmap, car_, rs).in_collision;
                }();
                if (analytic_collision) {
                    outcome.debug_event.outcome = "failed";
                    outcome.debug_event.detail = "waypoint_collision";
                    return outcome;
                }

                math::Pose2d wp_pose(wp[0], wp[1], math::Angle::from_radians(wp[2]));
                ResolvedPlannerBehavior resolved;
                {
                    common::ScopedProfilingTimer behavior_timer(
                        &frontier.profiling_collector, "planner.behavior_resolution");
                    resolved = PlannerBehaviorResolver::resolve(
                        wp_pose,
                        costmap,
                        node.frontier_id,
                        node.zone,
                        node.behavior_name,
                        selection.frontiers,
                        behavior_set_);
                }
                if (resolved.profile == nullptr ||
                    resolved.frontier_id < node.frontier_id) {
                    outcome.debug_event.outcome = "failed";
                    outcome.debug_event.detail = "path_validation_failed";
                    return outcome;
                }
                wp_resolutions.push_back(resolved);
            }

            const AnalyticPathValidationResult analytic_validation =
                validateAnalyticPathSegments(
                    node,
                    waypoints,
                    wp_resolutions,
                    global_max_same_motion_length_m,
                    node_context.yaw_bin_size_rad);
            if (!analytic_validation.valid) {
                outcome.debug_event.outcome = "failed";
                outcome.debug_event.detail =
                    analytic_validation.detail.empty()
                        ? "path_validation_failed"
                        : analytic_validation.detail;
                return outcome;
            }

            HybridAStarPlannerResult result;
            result.success = true;
            {
                common::ScopedProfilingTimer reconstruction_timer(
                    &frontier.profiling_collector, "planner.result_reconstruction");
                std::lock_guard<std::mutex> nodes_lock(nodes_mutex);
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

                for (size_t i = 1; i < waypoints.size(); ++i) {
                    prefix_poses.emplace_back(
                        waypoints[i][0],
                        waypoints[i][1],
                        math::Angle::from_radians(waypoints[i][2]));
                    prefix_behaviors.push_back(wp_resolutions[i].behavior_name);
                    prefix_directions.push_back(analytic_validation.segment_directions[i - 1]);
                }
                result.poses = std::move(prefix_poses);
                result.behavior_sequence = std::move(prefix_behaviors);
                result.segment_directions = std::move(prefix_directions);
            }
            result.detail = "Path found (analytic expansion).";
            outcome.result = std::move(result);
            outcome.debug_event.outcome = "success";
            return outcome;
        };

        while (true) {
            if (success_found.load() || shutdown.load()) {
                return;
            }
            if (Clock::now() >= deadline) {
                deadline_hit.store(true);
                shutdown.store(true);
                scheduler_cv.notify_all();
                return;
            }

            OpenEntry entry;
            bool have_entry = false;
            {
                std::lock_guard<std::mutex> frontier_lock(frontier.mutex);
                if (!frontier.open_queue.empty()) {
                    entry = frontier.open_queue.top();
                    frontier.open_queue.pop();
                    total_queue_entries.fetch_sub(1);
                    ++frontier.counters.expansions_popped;
                    ++frontier.debug_summary.popped;
                    if (frontier.debug_summary.first_pop_ms < 0.0) {
                        frontier.debug_summary.first_pop_ms =
                            elapsedMilliseconds(search_loop_start_time, Clock::now());
                    }
                    const auto best_it = frontier.best_g_by_key.find(entry.state_key);
                    if (best_it == frontier.best_g_by_key.end() ||
                        entry.g > best_it->second + 1e-6) {
                        ++frontier.counters.stale_entries_skipped;
                        ++frontier.debug_summary.stale_entries_skipped;
                    } else {
                        have_entry = true;
                    }
                }
            }

            if (!have_entry) {
                maybeSignalQuiescence();
                if (shutdown.load() || success_found.load()) {
                    return;
                }
                std::unique_lock<std::mutex> wait_lock(scheduler_mutex);
                scheduler_cv.wait_for(wait_lock, std::chrono::milliseconds(2));
                continue;
            }

            active_workers.fetch_add(1);

            SearchNode current_node;
            {
                std::lock_guard<std::mutex> nodes_lock(nodes_mutex);
                current_node = nodes[entry.node_index];
            }
            const SearchContext& current_context = *frontier.context;
            const PlannerBehaviorProfile& current_profile = *current_context.profile;

            PlannerExpansionDebugEvent expansion_debug;
            if (debug_trace != nullptr) {
                expansion_debug.expansion_index =
                    global_expansion_index.fetch_add(1);
                expansion_debug.node_index = entry.node_index;
                expansion_debug.frontier_id = frontier.descriptor.frontier_id;
                expansion_debug.frontier_role =
                    frontierRoleName(frontier.descriptor.role);
                expansion_debug.pose = current_node.pose;
                expansion_debug.zone_name = zoneLabel(current_node.zone);
                expansion_debug.behavior_name = current_node.behavior_name;
                expansion_debug.g = current_node.g;
                expansion_debug.h = current_node.h;
                expansion_debug.f = entry.f;
                expansion_debug.distance_to_goal_m =
                    std::hypot(request.goal.x - current_node.pose.x,
                               request.goal.y - current_node.pose.y);
                expansion_debug.heuristic_mode =
                    frontierHeuristicModeName(current_node.heuristic_mode);
                expansion_debug.stage_heuristic_value =
                    current_node.stage_heuristic_value;
                expansion_debug.final_goal_holonomic_value =
                    current_node.final_goal_holonomic_value;
                expansion_debug.nonholonomic_heuristic_value =
                    current_node.nonholonomic_heuristic_value;
                {
                    std::lock_guard<std::mutex> frontier_lock(frontier.mutex);
                    expansion_debug.open_queue_size_after_pop = frontier.open_queue.size();
                }
            }

            bool goal_satisfied = false;
            bool terminal_motion_valid = false;
            {
                common::ScopedProfilingTimer goal_check_timer(
                    &frontier.profiling_collector, "planner.goal_check");
                goal_satisfied = isGoalSatisfied(
                    current_node,
                    request.goal,
                    current_context.goal_distance_tolerance,
                    current_context.yaw_bin_size_rad);
                terminal_motion_valid = isTerminalMotionSegmentValid(current_node);
            }
            ++frontier.counters.goal_checks;
            if (debug_trace != nullptr) {
                expansion_debug.goal_satisfied = goal_satisfied;
                expansion_debug.terminal_motion_valid = terminal_motion_valid;
            }

            if (goal_satisfied && terminal_motion_valid) {
                ++frontier.counters.goal_hits;
                HybridAStarPlannerResult result;
                {
                    std::lock_guard<std::mutex> nodes_lock(nodes_mutex);
                    result = reconstructResult(
                        nodes,
                        static_cast<int>(entry.node_index),
                        &frontier.profiling_collector);
                }
                {
                    std::lock_guard<std::mutex> result_lock(result_mutex);
                    if (!shared_result.published) {
                        shared_result.result = std::move(result);
                        shared_result.published = true;
                        success_terminal_reason = "goal_satisfied";
                    }
                }
                if (debug_trace != nullptr) {
                    frontier.expansion_events.push_back(std::move(expansion_debug));
                }
                success_found.store(true);
                shutdown.store(true);
                active_workers.fetch_sub(1);
                scheduler_cv.notify_all();
                return;
            }

            ++local_expansion_count;
            const double distance_to_goal = std::hypot(
                request.goal.x - current_node.pose.x,
                request.goal.y - current_node.pose.y);
            const bool periodic_analytic_shot =
                current_profile.planner.analytic_shot &&
                local_expansion_count % current_context.analytic_expansion_interval == 0;
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
                ++frontier.counters.analytic_attempts;
                if (debug_trace != nullptr) {
                    expansion_debug.analytic_attempted = true;
                    expansion_debug.analytic_event = analytic_outcome.debug_event;
                }
                if (analytic_outcome.result.success) {
                    ++frontier.counters.analytic_successes;
                    {
                        std::lock_guard<std::mutex> result_lock(result_mutex);
                        if (!shared_result.published) {
                            shared_result.result = analytic_outcome.result;
                            shared_result.published = true;
                            success_terminal_reason = "analytic_expansion_success";
                        }
                    }
                    if (debug_trace != nullptr) {
                        frontier.expansion_events.push_back(std::move(expansion_debug));
                    }
                    success_found.store(true);
                    shutdown.store(true);
                    active_workers.fetch_sub(1);
                    scheduler_cv.notify_all();
                    return;
                }
                if (analytic_outcome.debug_event.detail == "no_ompl_spaces") {
                    ++frontier.counters.analytic_fail_no_ompl;
                } else if (analytic_outcome.debug_event.detail == "same_motion_guard") {
                    ++frontier.counters.analytic_fail_same_motion_guard;
                } else if (analytic_outcome.debug_event.detail == "path_length_exceeded" ||
                           analytic_outcome.debug_event.detail == "no_waypoints") {
                    ++frontier.counters.analytic_fail_path_length;
                } else if (analytic_outcome.debug_event.detail == "waypoint_out_of_bounds") {
                    ++frontier.counters.analytic_fail_out_of_bounds;
                } else if (analytic_outcome.debug_event.detail == "waypoint_collision") {
                    ++frontier.counters.analytic_fail_collision;
                } else {
                    ++frontier.counters.analytic_fail_validation;
                }
            }

            const auto projections = [&]() {
                common::ScopedProfilingTimer projections_timer(
                    &frontier.profiling_collector, "planner.motion_table_projections");
                return current_context.motion_table.getProjections(
                    static_cast<float>(current_node.pose.x / current_context.xy_resolution_m),
                    static_cast<float>(current_node.pose.y / current_context.xy_resolution_m),
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
                    if (heading_error < (2.0 * current_context.yaw_bin_size_rad)) {
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
                            ++frontier.counters.lane_following_candidates;
                            if (debug_trace != nullptr) {
                                expansion_debug.lane_following_candidate = true;
                            }
                        }
                    }
                }
            }

            const auto recordPrimitiveEvent =
                [&](PlannerPrimitiveDebugEvent primitive_event) {
                    if (debug_trace == nullptr) {
                        return;
                    }
                    if (primitive_event.outcome == "out_of_bounds") {
                        ++frontier.counters.primitive_out_of_bounds;
                    } else if (primitive_event.outcome == "cross_track_pruned") {
                        ++frontier.counters.primitive_cross_track_pruned;
                    } else if (primitive_event.outcome == "behavior_unresolved") {
                        ++frontier.counters.primitive_behavior_unresolved;
                    } else if (primitive_event.outcome == "primitive_disallowed") {
                        ++frontier.counters.primitive_disallowed;
                    } else if (primitive_event.outcome == "collision") {
                        ++frontier.counters.primitive_collision;
                    } else if (primitive_event.outcome == "motion_change_blocked") {
                        ++frontier.counters.primitive_motion_change_blocked;
                    } else if (primitive_event.outcome == "dominated") {
                        ++frontier.counters.primitive_dominated;
                    } else if (primitive_event.outcome == "enqueued") {
                        ++frontier.counters.primitive_enqueued;
                    }
                    expansion_debug.primitive_events.push_back(std::move(primitive_event));
                };

            const auto tryExpandPrimitive = [&](size_t primitive_idx) -> bool {
                common::ScopedProfilingTimer primitive_timer(
                    &frontier.profiling_collector, "planner.primitive_expansion_attempt");
                const auto& projection = projections[primitive_idx];
                unsigned int successor_heading_bin =
                    static_cast<unsigned int>(std::lround(projection.theta)) %
                    current_context.motion_table.getNumAngleBins();
                const double travel_m =
                    static_cast<double>(current_context.motion_table.getTravelCost(
                        static_cast<unsigned int>(primitive_idx))) *
                    current_context.xy_resolution_m;

                math::Pose2d successor_pose;
                successor_pose.x = static_cast<double>(projection.x) *
                    current_context.xy_resolution_m;
                successor_pose.y = static_cast<double>(projection.y) *
                    current_context.xy_resolution_m;
                successor_pose.theta = math::Angle::from_radians(
                    current_context.motion_table.getAngleFromBin(successor_heading_bin));

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
                        &frontier.profiling_collector, "planner.behavior_resolution");
                    return PlannerBehaviorResolver::resolve(
                        successor_pose,
                        costmap,
                        current_node.frontier_id,
                        current_node.zone,
                        current_node.behavior_name,
                        selection.frontiers,
                        behavior_set_);
                }();

                if (debug_trace != nullptr) {
                    primitive_event.resolved_behavior_name = resolved.behavior_name;
                    primitive_event.resolved_zone_name = zoneLabel(resolved.zone);
                }

                if (resolved.profile == nullptr) {
                    if (debug_trace != nullptr) {
                        primitive_event.outcome = "behavior_unresolved";
                        primitive_event.detail = "resolver_returned_null_profile";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }
                if (resolved.frontier_id < current_node.frontier_id) {
                    if (debug_trace != nullptr) {
                        primitive_event.outcome = "primitive_disallowed";
                        primitive_event.detail = "backward_frontier_transfer_not_supported";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }
                if (!isPrimitiveAllowed(projection.turn_dir, resolved)) {
                    if (debug_trace != nullptr) {
                        primitive_event.outcome = "primitive_disallowed";
                        primitive_event.detail = "primitive_not_allowed_for_behavior";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }

                const float successor_lethal_threshold = static_cast<float>(
                    resolved.profile->collision_checker.lethal_threshold);
                if (std::abs(successor_lethal_threshold -
                             default_collision_config.lethal_threshold) > 0.5f) {
                    CollisionCheckerConfig adjusted_config = default_collision_config;
                    adjusted_config.lethal_threshold = successor_lethal_threshold;
                    CollisionChecker adjusted_checker(adjusted_config);
                    const bool in_collision = [&]() {
                        common::ScopedProfilingTimer collision_timer(
                            &frontier.profiling_collector,
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
                            &frontier.profiling_collector,
                            "planner.collision_check.primitive_default");
                        return default_collision_checker.checkCollision(
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
                        &frontier.profiling_collector, "planner.edge_cost");
                    return computeEdgeCost(
                               costmap,
                               *resolved.profile,
                               current_node,
                               successor_position,
                               successor_pose.theta.radians(),
                               projection.turn_dir,
                               travel_m) +
                        computeGoalApproachStraightPenalty(
                            *resolved.profile,
                            successor_pose,
                            request.goal,
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

                const FrontierRuntime& destination_frontier =
                    *frontier_runtimes[resolved.frontier_id];
                const SearchContext& destination_context = *destination_frontier.context;
                if (resolved.frontier_id != current_node.frontier_id) {
                    successor_heading_bin =
                        destination_context.motion_table.getClosestAngularBin(
                            normalizeAngleUnsigned(successor_pose.theta.radians()));
                }

                const FrontierHeuristicEvaluation successor_heuristic =
                    computeFrontierHeuristic(
                        costmap,
                        destination_frontier,
                        heuristic,
                        selectHeuristicModel(resolved),
                        successor_pose,
                        request.goal,
                        &frontier.profiling_collector);
                const double successor_h = successor_heuristic.combined_h;
                SearchNode successor_node;
                successor_node.pose = successor_pose;
                successor_node.frontier_id = resolved.frontier_id;
                successor_node.heading_bin = successor_heading_bin;
                successor_node.g = new_g;
                successor_node.h = successor_h;
                successor_node.parent_index = static_cast<int>(entry.node_index);
                successor_node.zone = resolved.zone;
                successor_node.behavior_name = resolved.behavior_name;
                successor_node.last_turn_class = classifyTurn(projection.turn_dir);
                successor_node.has_inbound_motion = true;
                successor_node.inbound_motion = successor_motion;
                successor_node.same_motion_length_m = successor_same_motion_length_m;
                successor_node.same_motion_remaining_to_change_m =
                    successor_same_motion_remaining_to_change_m;
                successor_node.heuristic_mode = successor_heuristic.mode;
                successor_node.stage_heuristic_value =
                    successor_heuristic.stage_heuristic_value;
                successor_node.final_goal_holonomic_value =
                    successor_heuristic.final_goal_holonomic_value;
                successor_node.nonholonomic_heuristic_value =
                    successor_heuristic.nonholonomic_heuristic_value;

                const DiscreteStateKey successor_key = discretizeState(
                    successor_pose,
                    successor_heading_bin,
                    successor_node,
                    destination_context.xy_resolution_m,
                    global_max_same_motion_length_m);
                const auto successor_index = enqueueSuccessor(
                    resolved.frontier_id,
                    std::move(successor_node),
                    successor_key,
                    successor_h,
                    new_g);
                if (!successor_index.has_value()) {
                    if (debug_trace != nullptr) {
                        primitive_event.edge_cost = edge_cost;
                        primitive_event.new_g = new_g;
                        primitive_event.successor_h = successor_h;
                        primitive_event.outcome = "dominated";
                        primitive_event.detail =
                            "existing_state_has_better_or_equal_cost";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }

                if (resolved.frontier_id != current_node.frontier_id) {
                    recordHandoff(current_node.frontier_id, resolved.frontier_id);
                }
                if (debug_trace != nullptr) {
                    primitive_event.edge_cost = edge_cost;
                    primitive_event.new_g = new_g;
                    primitive_event.successor_h = successor_h;
                    primitive_event.outcome = "enqueued";
                    primitive_event.detail =
                        resolved.frontier_id == current_node.frontier_id
                            ? "successor_added_to_frontier_queue"
                            : "successor_handed_off_to_next_frontier";
                    recordPrimitiveEvent(std::move(primitive_event));
                }
                return true;
            };

            if (lane_following_node &&
                forward_primitive_idx < projections.size() &&
                tryExpandPrimitive(forward_primitive_idx)) {
                ++frontier.counters.lane_suppression_forward_only_applied;
                if (debug_trace != nullptr) {
                    expansion_debug.lane_suppression_forward_success = true;
                    frontier.expansion_events.push_back(std::move(expansion_debug));
                }
                active_workers.fetch_sub(1);
                maybeSignalQuiescence();
                continue;
            }

            if (lane_following_node && forward_primitive_idx < projections.size()) {
                ++frontier.counters.lane_suppression_fallbacks;
                if (debug_trace != nullptr) {
                    expansion_debug.lane_suppression_fallback = true;
                }
            }

            for (size_t primitive_idx = 0; primitive_idx < projections.size(); ++primitive_idx) {
                tryExpandPrimitive(primitive_idx);
            }

            if (debug_trace != nullptr) {
                frontier.expansion_events.push_back(std::move(expansion_debug));
            }
            active_workers.fetch_sub(1);
            maybeSignalQuiescence();
        }
    };

    std::vector<std::thread> workers;
    workers.reserve(frontier_runtimes.size());
    for (size_t frontier_index = 0; frontier_index < frontier_runtimes.size(); ++frontier_index) {
        workers.emplace_back(worker, frontier_index);
    }
    for (auto& worker_thread : workers) {
        worker_thread.join();
    }

    if (shared_result.published) {
        if (debug_trace != nullptr) {
            debug_trace->terminal_reason = success_terminal_reason;
        }
        return finalizeResult(shared_result.result, nodes, total_queue_peak_size.load());
    }

    if (deadline_hit.load()) {
        if (debug_trace != nullptr) {
            debug_trace->terminal_reason =
                "Hybrid A* search timed out before finding a path.";
        }
        failure_result.detail = "Hybrid A* search timed out before finding a path.";
        return finalizeResult(failure_result, nodes, total_queue_peak_size.load());
    }

    if (debug_trace != nullptr) {
        debug_trace->terminal_reason =
            "Hybrid A* exhausted the frontier queues without finding a path.";
    }
    failure_result.detail =
        "Hybrid A* exhausted the frontier queues without finding a path.";
    return finalizeResult(failure_result, nodes, total_queue_peak_size.load());
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
