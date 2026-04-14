#include "coastmotionplanning/planning/planner_behavior_resolver.hpp"

#include <cmath>

#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/costs/zone_selector.hpp"

namespace coastmotionplanning {
namespace planning {

namespace {

struct ZoneConstraintLookup {
    enum class Kind {
        CurrentZone,
        Transition,
        IndexedZone
    };

    Kind kind{Kind::CurrentZone};
    size_t zone_index{0};
};

ZoneConstraintLookup lookupZoneConstraint(
    const math::Pose2d& successor_pose,
    const grid_map::GridMap& costmap,
    size_t candidate_zone_count) {
    ZoneConstraintLookup result;

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
    if (std::isnan(zone_value)) {
        return result;
    }

    if (std::abs(zone_value - costs::ZoneConstraintValues::ZONE_TRANSITION) < 0.5f ||
        std::abs(zone_value - costs::ZoneConstraintValues::ZONE_NONE) < 0.5f) {
        result.kind = ZoneConstraintLookup::Kind::Transition;
        return result;
    }

    const long zone_index = std::lround(zone_value);
    if (zone_index >= 0 &&
        static_cast<size_t>(zone_index) < candidate_zone_count &&
        std::abs(zone_value - static_cast<float>(zone_index)) < 0.5f) {
        result.kind = ZoneConstraintLookup::Kind::IndexedZone;
        result.zone_index = static_cast<size_t>(zone_index);
    }

    return result;
}

} // namespace

ResolvedPlannerBehavior PlannerBehaviorResolver::resolve(
    const math::Pose2d& successor_pose,
    const grid_map::GridMap& costmap,
    const std::shared_ptr<zones::Zone>& current_zone,
    const std::string& current_behavior_name,
    const std::vector<std::shared_ptr<zones::Zone>>& candidate_zones,
    const PlannerBehaviorSet& behavior_set) {
    const ZoneConstraintLookup lookup =
        lookupZoneConstraint(successor_pose, costmap, candidate_zones.size());

    if (lookup.kind != ZoneConstraintLookup::Kind::IndexedZone) {
        return ResolvedPlannerBehavior{
            current_zone,
            current_behavior_name,
            &behavior_set.get(current_behavior_name),
            false
        };
    }

    const auto& containing_zone = candidate_zones[lookup.zone_index];
    if (containing_zone == nullptr || containing_zone == current_zone) {
        return ResolvedPlannerBehavior{
            current_zone,
            current_behavior_name,
            &behavior_set.get(current_behavior_name),
            false
        };
    }

    const std::string resolved_behavior_name = containing_zone->getResolvedPlannerBehavior();
    if (resolved_behavior_name.empty()) {
        return ResolvedPlannerBehavior{
            containing_zone,
            current_behavior_name,
            &behavior_set.get(current_behavior_name),
            true
        };
    }

    return ResolvedPlannerBehavior{
        containing_zone,
        resolved_behavior_name,
        &behavior_set.get(resolved_behavior_name),
        true
    };
}

} // namespace planning
} // namespace coastmotionplanning
