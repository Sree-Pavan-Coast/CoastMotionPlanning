#pragma once

#include <memory>
#include <string>
#include <vector>

#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/planning/planner_behavior_profile.hpp"
#include "coastmotionplanning/planning/planner_behavior_set.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace planning {

struct ResolvedPlannerBehavior {
    std::shared_ptr<zones::Zone> zone;
    std::string behavior_name;
    const PlannerBehaviorProfile* profile{nullptr};
    bool switched_zone{false};
};

class PlannerBehaviorResolver {
public:
    static ResolvedPlannerBehavior resolve(
        const math::Pose2d& successor_pose,
        const std::shared_ptr<zones::Zone>& current_zone,
        const std::string& current_behavior_name,
        const std::vector<std::shared_ptr<zones::Zone>>& candidate_zones,
        const PlannerBehaviorSet& behavior_set);
};

} // namespace planning
} // namespace coastmotionplanning
