#include "coastmotionplanning/planning/planner_behavior_resolver.hpp"

#include <cmath>

#include "coastmotionplanning/common/math_constants.hpp"
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/zones/zone_type_utils.hpp"

namespace coastmotionplanning {
namespace planning {

namespace {

struct FrontierConstraintLookup {
    enum class Kind {
        CurrentFrontier,
        IndexedFrontier
    };

    Kind kind{Kind::CurrentFrontier};
    size_t frontier_id{0};
};

struct TransitionMetrics {
    bool valid{false};
    double track_station_m{std::numeric_limits<double>::quiet_NaN()};
    double lane_distance_m{std::numeric_limits<double>::infinity()};
    double lane_heading_error_rad{std::numeric_limits<double>::quiet_NaN()};
};

double normalizeAngleSigned(double angle) {
    angle = std::fmod(angle + common::PI, common::TWO_PI);
    if (angle < 0.0) {
        angle += common::TWO_PI;
    }
    return angle - common::PI;
}

FrontierConstraintLookup lookupFrontierConstraint(
    const math::Pose2d& successor_pose,
    const grid_map::GridMap& costmap,
    size_t frontier_count) {
    FrontierConstraintLookup result;

    if (!costmap.exists(costs::CostmapLayerNames::ZONE_CONSTRAINTS)) {
        return result;
    }

    const grid_map::Position successor_position(successor_pose.x, successor_pose.y);
    if (!costmap.isInside(successor_position)) {
        return result;
    }

    const float zone_value = costmap.atPosition(
        costs::CostmapLayerNames::ZONE_CONSTRAINTS,
        successor_position,
        grid_map::InterpolationMethods::INTER_NEAREST);
    if (std::isnan(zone_value) ||
        std::abs(zone_value - costs::ZoneConstraintValues::ZONE_NONE) < 0.5f) {
        return result;
    }

    const long frontier_id = std::lround(zone_value);
    if (frontier_id >= 0 &&
        static_cast<size_t>(frontier_id) < frontier_count &&
        std::abs(zone_value - static_cast<float>(frontier_id)) < 0.5f) {
        result.kind = FrontierConstraintLookup::Kind::IndexedFrontier;
        result.frontier_id = static_cast<size_t>(frontier_id);
    }

    return result;
}

std::string resolvedFrontierBehaviorName(
    const costs::SearchFrontierDescriptor& frontier,
    const std::string& fallback_behavior_name) {
    return frontier.behavior_name.empty() ? fallback_behavior_name : frontier.behavior_name;
}

TransitionMetrics readTransitionMetrics(const math::Pose2d& successor_pose,
                                       const grid_map::GridMap& costmap,
                                       size_t frontier_id) {
    TransitionMetrics metrics;
    if (!costmap.exists(costs::CostmapLayerNames::ZONE_CONSTRAINTS) ||
        !costmap.exists(costs::CostmapLayerNames::TRACK_STATION) ||
        !costmap.exists(costs::CostmapLayerNames::LANE_DISTANCE) ||
        !costmap.exists(costs::CostmapLayerNames::LANE_HEADING)) {
        return metrics;
    }

    const grid_map::Position successor_position(successor_pose.x, successor_pose.y);
    if (!costmap.isInside(successor_position)) {
        return metrics;
    }

    const float owner = costmap.atPosition(
        costs::CostmapLayerNames::ZONE_CONSTRAINTS,
        successor_position,
        grid_map::InterpolationMethods::INTER_NEAREST);
    if (!std::isfinite(owner) ||
        std::abs(owner - static_cast<float>(frontier_id)) >= 0.5f) {
        return metrics;
    }

    const float station = costmap.atPosition(
        costs::CostmapLayerNames::TRACK_STATION,
        successor_position,
        grid_map::InterpolationMethods::INTER_NEAREST);
    const float lane_distance = costmap.atPosition(
        costs::CostmapLayerNames::LANE_DISTANCE,
        successor_position,
        grid_map::InterpolationMethods::INTER_NEAREST);
    const float lane_heading = costmap.atPosition(
        costs::CostmapLayerNames::LANE_HEADING,
        successor_position,
        grid_map::InterpolationMethods::INTER_NEAREST);
    if (!std::isfinite(station) ||
        !std::isfinite(lane_distance) ||
        !std::isfinite(lane_heading)) {
        return metrics;
    }

    metrics.valid = true;
    metrics.track_station_m = static_cast<double>(station);
    metrics.lane_distance_m = static_cast<double>(lane_distance);
    metrics.lane_heading_error_rad = std::abs(normalizeAngleSigned(
        successor_pose.theta.radians() - static_cast<double>(lane_heading)));
    return metrics;
}

ResolvedPlannerBehavior makeResolvedBehavior(
    size_t frontier_id,
    const std::shared_ptr<zones::Zone>& zone,
    const std::string& behavior_name,
    const PlannerBehaviorSet& behavior_set,
    bool switched_zone,
    bool switched_frontier,
    const std::string& steady_behavior_name,
    const TransitionMetrics& metrics,
    const ActiveZoneTransitionState& active_transition,
    const std::string& transition_promotion_reason = "") {
    ResolvedPlannerBehavior resolved;
    resolved.frontier_id = frontier_id;
    resolved.zone = zone;
    resolved.behavior_name = behavior_name;
    resolved.profile = &behavior_set.get(behavior_name);
    resolved.switched_zone = switched_zone;
    resolved.switched_frontier = switched_frontier;
    resolved.active_transition = active_transition;
    resolved.steady_behavior_name = steady_behavior_name.empty()
        ? behavior_name
        : steady_behavior_name;
    resolved.transition_track_station_m = metrics.track_station_m;
    resolved.transition_lane_distance_m = metrics.lane_distance_m;
    resolved.transition_lane_heading_error_rad = metrics.lane_heading_error_rad;
    resolved.transition_promotion_reason = transition_promotion_reason;
    return resolved;
}

} // namespace

ResolvedPlannerBehavior PlannerBehaviorResolver::resolve(
    const math::Pose2d& successor_pose,
    const grid_map::GridMap& costmap,
    size_t current_frontier_id,
    const std::shared_ptr<zones::Zone>& current_zone,
    const std::string& current_behavior_name,
    const std::vector<costs::SearchFrontierDescriptor>& frontiers,
    const PlannerBehaviorSet& behavior_set,
    const ActiveZoneTransitionState& active_transition) {
    const FrontierConstraintLookup lookup =
        lookupFrontierConstraint(successor_pose, costmap, frontiers.size());

    if (lookup.kind != FrontierConstraintLookup::Kind::IndexedFrontier) {
        return makeResolvedBehavior(
            current_frontier_id,
            current_zone,
            current_behavior_name,
            behavior_set,
            false,
            false,
            current_behavior_name,
            TransitionMetrics{},
            active_transition);
    }

    const auto& frontier = frontiers[lookup.frontier_id];
    const std::string steady_behavior_name =
        resolvedFrontierBehaviorName(frontier, current_behavior_name);
    const TransitionMetrics metrics =
        readTransitionMetrics(successor_pose, costmap, frontier.frontier_id);

    if (active_transition.isActive() &&
        frontier.frontier_id == current_frontier_id &&
        frontier.zone != nullptr &&
        zones::zoneTypeName(frontier.zone) == active_transition.policy->to_zone_type) {
        const double depth_m =
            metrics.valid
                ? std::abs(metrics.track_station_m - active_transition.entry_station_m)
                : 0.0;
        const double heading_error_threshold_rad =
            active_transition.policy->heading_error_threshold_deg * common::DEG_TO_RAD;

        if (metrics.valid && depth_m >= active_transition.policy->max_depth_m) {
            return makeResolvedBehavior(
                frontier.frontier_id,
                frontier.zone,
                steady_behavior_name,
                behavior_set,
                frontier.zone != nullptr && frontier.zone != current_zone,
                frontier.frontier_id != current_frontier_id,
                steady_behavior_name,
                metrics,
                {},
                "max_depth_forced");
        }

        if (metrics.valid &&
            depth_m >= active_transition.policy->min_depth_m &&
            metrics.lane_distance_m <=
                active_transition.policy->lane_distance_threshold_m &&
            metrics.lane_heading_error_rad <= heading_error_threshold_rad) {
            return makeResolvedBehavior(
                frontier.frontier_id,
                frontier.zone,
                steady_behavior_name,
                behavior_set,
                frontier.zone != nullptr && frontier.zone != current_zone,
                frontier.frontier_id != current_frontier_id,
                steady_behavior_name,
                metrics,
                {},
                "aligned_and_deep_enough");
        }

        return makeResolvedBehavior(
            frontier.frontier_id,
            frontier.zone,
            active_transition.policy->entry_behavior,
            behavior_set,
            frontier.zone != nullptr && frontier.zone != current_zone,
            frontier.frontier_id != current_frontier_id,
            steady_behavior_name,
            metrics,
            active_transition);
    }

    if (frontier.frontier_id != current_frontier_id &&
        current_zone != nullptr &&
        frontier.zone != nullptr) {
        const auto* policy = behavior_set.findTransitionPolicy(
            zones::zoneTypeName(current_zone),
            zones::zoneTypeName(frontier.zone));
        if (policy != nullptr && metrics.valid) {
            return makeResolvedBehavior(
                frontier.frontier_id,
                frontier.zone,
                policy->entry_behavior,
                behavior_set,
                frontier.zone != nullptr && frontier.zone != current_zone,
                frontier.frontier_id != current_frontier_id,
                steady_behavior_name,
                metrics,
                ActiveZoneTransitionState{
                    policy,
                    current_frontier_id,
                    metrics.track_station_m
                });
        }
    }

    return makeResolvedBehavior(
        frontier.frontier_id,
        frontier.zone,
        steady_behavior_name,
        behavior_set,
        frontier.zone != nullptr && frontier.zone != current_zone,
        frontier.frontier_id != current_frontier_id,
        steady_behavior_name,
        metrics,
        {});
}

} // namespace planning
} // namespace coastmotionplanning
