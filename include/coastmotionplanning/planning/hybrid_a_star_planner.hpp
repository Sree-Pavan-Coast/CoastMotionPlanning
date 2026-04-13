#pragma once

#include <memory>
#include <string>
#include <vector>

#include "coastmotionplanning/common/types.hpp"
#include "coastmotionplanning/costs/dual_model_non_holonomic_heuristic.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/motion_primitives/motion_primitive_types.hpp"
#include "coastmotionplanning/planning/planner_behavior_resolver.hpp"
#include "coastmotionplanning/planning/planner_behavior_set.hpp"
#include "coastmotionplanning/robot/car.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace planning {

struct HybridAStarPlannerRequest {
    math::Pose2d start;
    math::Pose2d goal;
    std::string initial_behavior_name;
    std::string dual_model_lut_path;
};

struct HybridAStarPlannerResult {
    bool success{false};
    std::string detail;
    std::vector<math::Pose2d> poses;
    std::vector<std::string> behavior_sequence;
    std::vector<common::MotionDirection> segment_directions;
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
