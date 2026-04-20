#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "coastmotionplanning/common/profiling.hpp"
#include "coastmotionplanning/common/types.hpp"
#include "coastmotionplanning/costs/dual_model_non_holonomic_heuristic.hpp"
#include "coastmotionplanning/geometry/shape_types.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/motion_primitives/motion_primitive_types.hpp"
#include "coastmotionplanning/planning/planner_behavior_resolver.hpp"
#include "coastmotionplanning/planning/planner_behavior_set.hpp"
#include "coastmotionplanning/robot/car.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace planning {

struct PlannerPrimitiveDebugEvent {
    size_t primitive_index{0};
    std::string turn_direction;
    math::Pose2d successor_pose;
    std::string outcome;
    std::string detail;
    std::string resolved_behavior_name;
    std::string resolved_zone_name;
    double travel_m{0.0};
    double edge_cost{0.0};
    double new_g{0.0};
    double successor_h{0.0};
};

struct PlannerAnalyticDebugEvent {
    bool attempted{false};
    std::string trigger;
    double distance_to_goal_m{0.0};
    double path_length_m{0.0};
    size_t waypoint_count{0};
    std::string outcome;
    std::string detail;
};

struct PlannerExpansionDebugEvent {
    size_t expansion_index{0};
    size_t node_index{0};
    size_t frontier_id{0};
    std::string frontier_role;
    math::Pose2d pose;
    std::string zone_name;
    std::string behavior_name;
    double g{0.0};
    double h{0.0};
    double f{0.0};
    double distance_to_goal_m{0.0};
    size_t open_queue_size_after_pop{0};
    bool goal_satisfied{false};
    bool terminal_motion_valid{false};
    bool lane_following_candidate{false};
    bool lane_suppression_forward_success{false};
    bool lane_suppression_fallback{false};
    bool analytic_attempted{false};
    PlannerAnalyticDebugEvent analytic_event;
    std::vector<PlannerPrimitiveDebugEvent> primitive_events;
};

struct PlannerFrontierDebugSummary {
    size_t frontier_id{0};
    std::string frontier_role;
    std::string zone_name;
    std::string behavior_name;
    uint64_t enqueued{0};
    uint64_t popped{0};
    uint64_t stale_entries_skipped{0};
    size_t closed_set_size{0};
    size_t open_queue_peak_size{0};
    double first_enqueue_ms{-1.0};
    double first_pop_ms{-1.0};
    std::vector<common::ProfilingScopeSummary> profiling_scopes;
};

struct PlannerFrontierHandoffDebugSummary {
    size_t from_frontier_id{0};
    size_t to_frontier_id{0};
    uint64_t transfer_count{0};
    double first_transfer_ms{-1.0};
};

struct HybridAStarPlannerDebugTrace {
    std::string initial_behavior_name;
    std::string start_zone_name;
    std::string goal_zone_name;
    std::vector<std::string> selected_zone_names;
    double zone_selection_ms{0.0};
    double costmap_build_ms{0.0};
    double heuristic_setup_ms{0.0};
    double search_loop_ms{0.0};
    double total_planning_ms{0.0};
    size_t nodes_allocated{0};
    size_t unique_state_count{0};
    size_t open_queue_peak_size{0};
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
    std::string terminal_reason;
    std::vector<common::ProfilingScopeSummary> profiling_scopes;
    std::vector<PlannerFrontierDebugSummary> frontier_summaries;
    std::vector<PlannerFrontierHandoffDebugSummary> frontier_handoffs;
    std::vector<PlannerExpansionDebugEvent> expansions;
};

struct HybridAStarPlannerRequest {
    math::Pose2d start;
    math::Pose2d goal;
    std::vector<geometry::Polygon2d> obstacle_polygons;
    std::string initial_behavior_name;
    std::string dual_model_lut_path;
};

struct HybridAStarPlannerResult {
    bool success{false};
    std::string detail;
    std::vector<math::Pose2d> poses;
    std::vector<std::string> behavior_sequence;
    std::vector<common::MotionDirection> segment_directions;
    std::shared_ptr<HybridAStarPlannerDebugTrace> debug_trace;
};

class HybridAStarPlanner {
public:
    HybridAStarPlanner(const robot::Car& car,
                       std::vector<std::shared_ptr<zones::Zone>> all_zones,
                       PlannerBehaviorSet behavior_set);

    HybridAStarPlannerResult plan(const HybridAStarPlannerRequest& request) const;

    static costs::HeuristicModel selectHeuristicModel(
        const ResolvedPlannerBehavior& resolved_behavior);
    static bool isPrimitiveAllowed(
        motion_primitives::TurnDirection turn_direction,
        const ResolvedPlannerBehavior& resolved_behavior);

private:
    const robot::Car& car_;
    std::vector<std::shared_ptr<zones::Zone>> all_zones_;
    PlannerBehaviorSet behavior_set_;
};

} // namespace planning
} // namespace coastmotionplanning
