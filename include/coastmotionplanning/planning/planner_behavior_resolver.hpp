#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>

#include "coastmotionplanning/costs/zone_selector.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/planning/planner_behavior_profile.hpp"
#include "coastmotionplanning/planning/planner_behavior_set.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace planning {

struct ResolvedPlannerBehavior {
    size_t frontier_id{0};
    std::shared_ptr<zones::Zone> zone;
    std::string behavior_name;
    const PlannerBehaviorProfile* profile{nullptr};
    bool switched_zone{false};
    bool switched_frontier{false};
    ActiveZoneTransitionState active_transition;
    std::string steady_behavior_name;
    double transition_track_station_m{std::numeric_limits<double>::quiet_NaN()};
    double transition_lane_distance_m{std::numeric_limits<double>::infinity()};
    double transition_lane_heading_error_rad{std::numeric_limits<double>::quiet_NaN()};
    std::string transition_promotion_reason;
};

class PlannerBehaviorResolver {
public:
    static ResolvedPlannerBehavior resolve(
        const math::Pose2d& successor_pose,
        const grid_map::GridMap& costmap,
        size_t current_frontier_id,
        const std::shared_ptr<zones::Zone>& current_zone,
        const std::string& current_behavior_name,
        const std::vector<costs::SearchFrontierDescriptor>& frontiers,
        const PlannerBehaviorSet& behavior_set,
        const ActiveZoneTransitionState& active_transition = {});
};

} // namespace planning
} // namespace coastmotionplanning
