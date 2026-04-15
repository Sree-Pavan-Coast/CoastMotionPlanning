#include "coastmotionplanning/planning/planner_behavior_resolver.hpp"

#include <cmath>

#include "coastmotionplanning/costs/costmap_types.hpp"

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

} // namespace

ResolvedPlannerBehavior PlannerBehaviorResolver::resolve(
    const math::Pose2d& successor_pose,
    const grid_map::GridMap& costmap,
    size_t current_frontier_id,
    const std::shared_ptr<zones::Zone>& current_zone,
    const std::string& current_behavior_name,
    const std::vector<costs::SearchFrontierDescriptor>& frontiers,
    const PlannerBehaviorSet& behavior_set) {
    const FrontierConstraintLookup lookup =
        lookupFrontierConstraint(successor_pose, costmap, frontiers.size());

    if (lookup.kind != FrontierConstraintLookup::Kind::IndexedFrontier) {
        return ResolvedPlannerBehavior{
            current_frontier_id,
            current_zone,
            current_behavior_name,
            &behavior_set.get(current_behavior_name),
            false,
            false
        };
    }

    const auto& frontier = frontiers[lookup.frontier_id];
    const std::string resolved_behavior_name =
        frontier.behavior_name.empty() ? current_behavior_name : frontier.behavior_name;

    return ResolvedPlannerBehavior{
        frontier.frontier_id,
        frontier.zone,
        resolved_behavior_name,
        &behavior_set.get(resolved_behavior_name),
        frontier.zone != nullptr && frontier.zone != current_zone,
        frontier.frontier_id != current_frontier_id
    };
}

} // namespace planning
} // namespace coastmotionplanning
