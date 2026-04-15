#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "coastmotionplanning/planning/planner_behavior_set.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace planning {

enum class PlanningIntent {
    NORMAL,
    TIGHT_MANEUVER
};

struct PlanningRequestContext {
    PlanningIntent intent{PlanningIntent::NORMAL};
    std::shared_ptr<zones::Zone> start_zone;
    std::shared_ptr<zones::Zone> goal_zone;
};

struct PlanningAttempt {
    std::string profile;
    std::string transition_profile;
    size_t attempt_index{0};
};

struct PlannerRunResult {
    bool success{false};
    std::string detail;
};

using PlannerAttemptRunner = std::function<PlannerRunResult(const PlanningAttempt&)>;

struct BehaviorTreePlanResult {
    bool success{false};
    std::string preferred_profile;
    std::string selected_profile;
    std::string detail;
    std::vector<std::string> attempted_profiles;
};

class BehaviorTreePlannerOrchestrator {
public:
    BehaviorTreePlannerOrchestrator(std::string xml_filepath,
                                    PlannerBehaviorSet behavior_set);

    BehaviorTreePlanResult run(const PlanningRequestContext& request,
                               const PlannerAttemptRunner& runner) const;

private:
    std::string xml_filepath_;
    PlannerBehaviorSet behavior_set_;
};

std::string toString(PlanningIntent intent);

} // namespace planning
} // namespace coastmotionplanning
