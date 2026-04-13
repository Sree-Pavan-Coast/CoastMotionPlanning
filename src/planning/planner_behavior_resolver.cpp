#include "coastmotionplanning/planning/planner_behavior_resolver.hpp"

#include "coastmotionplanning/costs/zone_selector.hpp"

namespace coastmotionplanning {
namespace planning {

ResolvedPlannerBehavior PlannerBehaviorResolver::resolve(
    const math::Pose2d& successor_pose,
    const std::shared_ptr<zones::Zone>& current_zone,
    const std::string& current_behavior_name,
    const std::vector<std::shared_ptr<zones::Zone>>& candidate_zones,
    const PlannerBehaviorSet& behavior_set) {
    const geometry::Point2d successor_point(successor_pose.x, successor_pose.y);
    const auto containing_zone =
        costs::ZoneSelector::findContainingZone(successor_point, candidate_zones);

    if (containing_zone == nullptr || containing_zone == current_zone) {
        return ResolvedPlannerBehavior{
            current_zone,
            current_behavior_name,
            &behavior_set.get(current_behavior_name),
            false
        };
    }

    const std::string resolved_behavior_name = containing_zone->getResolvedPlannerBehavior();
    return ResolvedPlannerBehavior{
        containing_zone,
        resolved_behavior_name,
        &behavior_set.get(resolved_behavior_name),
        true
    };
}

} // namespace planning
} // namespace coastmotionplanning
