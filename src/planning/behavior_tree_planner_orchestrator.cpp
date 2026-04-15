#include "coastmotionplanning/planning/behavior_tree_planner_orchestrator.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include <behaviortree_cpp/bt_factory.h>

#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

namespace coastmotionplanning {
namespace planning {

namespace {

constexpr const char* kPlanningIntentKey = "planning_intent";
constexpr const char* kStartZoneTypeKey = "start_zone_type";
constexpr const char* kGoalZoneTypeKey = "goal_zone_type";
constexpr const char* kStartZoneBehaviorKey = "start_zone_behavior";
constexpr const char* kGoalZoneBehaviorKey = "goal_zone_behavior";
constexpr const char* kPreferredProfileKey = "preferred_profile";
constexpr const char* kSelectedProfileKey = "selected_profile";
constexpr const char* kTransitionProfileKey = "transition_profile";
constexpr const char* kAttemptIndexKey = "attempt_index";
constexpr const char* kPlanSucceededKey = "plan_succeeded";
constexpr const char* kRuntimeStateKey = "runtime_state";

constexpr const char* kRelaxedProfile = "relaxed_profile";

struct RuntimeState {
    PlannerAttemptRunner runner;
    const PlannerBehaviorSet* behavior_set{nullptr};
    size_t attempt_index{0};
    bool plan_succeeded{false};
    std::string detail;
    std::vector<std::string> attempted_profiles;
};

BT::Blackboard* blackboard(BT::TreeNode& node) {
    const auto& config = static_cast<const BT::TreeNode&>(node).config();
    if (!config.blackboard) {
        throw std::runtime_error("Behavior tree node is missing a blackboard.");
    }
    return config.blackboard->rootBlackboard();
}

std::string zoneTypeName(const std::shared_ptr<zones::Zone>& zone) {
    if (!zone) {
        return "";
    }
    if (dynamic_cast<const zones::TrackMainRoad*>(zone.get()) != nullptr) {
        return "TrackMainRoad";
    }
    if (dynamic_cast<const zones::ManeuveringZone*>(zone.get()) != nullptr) {
        return "ManeuveringZone";
    }
    return "UnknownZone";
}

std::string zoneBehaviorName(const std::shared_ptr<zones::Zone>& zone) {
    if (!zone || !zone->getPlannerBehavior().has_value()) {
        return "";
    }
    return zone->getPlannerBehavior().value();
}

std::shared_ptr<RuntimeState> runtimeState(BT::TreeNode& node) {
    return blackboard(node)->get<std::shared_ptr<RuntimeState>>(kRuntimeStateKey);
}

BT::NodeStatus setPreferredProfile(BT::TreeNode& node, const std::string& profile) {
    auto state = runtimeState(node);
    if (!state->behavior_set->contains(profile)) {
        state->detail = "Planner behavior '" + profile + "' is not defined.";
        return BT::NodeStatus::FAILURE;
    }

    blackboard(node)->set<std::string>(kPreferredProfileKey, profile);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus useAssignedBehavior(BT::TreeNode& node, const std::string& blackboard_key) {
    const std::string profile = blackboard(node)->get<std::string>(blackboard_key);
    if (profile.empty()) {
        return BT::NodeStatus::FAILURE;
    }
    return setPreferredProfile(node, profile);
}

void registerPlannerNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerSimpleCondition(
        "IsTightManeuver",
        [](BT::TreeNode& node) {
            const std::string intent = blackboard(node)->get<std::string>(kPlanningIntentKey);
            return intent == "tight_maneuver" ? BT::NodeStatus::SUCCESS
                                               : BT::NodeStatus::FAILURE;
        });

    factory.registerSimpleCondition(
        "HasGoalAssignedBehavior",
        [](BT::TreeNode& node) {
            const std::string behavior = blackboard(node)->get<std::string>(kGoalZoneBehaviorKey);
            return behavior.empty() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        });

    factory.registerSimpleAction(
        "UseGoalAssignedBehavior",
        [](BT::TreeNode& node) {
            return useAssignedBehavior(node, kGoalZoneBehaviorKey);
        });

    factory.registerSimpleCondition(
        "HasStartAssignedBehavior",
        [](BT::TreeNode& node) {
            const std::string behavior = blackboard(node)->get<std::string>(kStartZoneBehaviorKey);
            return behavior.empty() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        });

    factory.registerSimpleAction(
        "UseStartAssignedBehavior",
        [](BT::TreeNode& node) {
            return useAssignedBehavior(node, kStartZoneBehaviorKey);
        });

    factory.registerSimpleCondition(
        "HasStartZoneType",
        [](BT::TreeNode& node) {
            const auto expected = node.getInput<std::string>("zone_type");
            if (!expected) {
                throw BT::RuntimeError(expected.error());
            }

            const std::string start_zone_type =
                blackboard(node)->get<std::string>(kStartZoneTypeKey);
            return start_zone_type == expected.value() ? BT::NodeStatus::SUCCESS
                                                       : BT::NodeStatus::FAILURE;
        },
        {BT::InputPort<std::string>("zone_type")});

    factory.registerSimpleCondition(
        "GoalZoneTypeEquals",
        [](BT::TreeNode& node) {
            const auto expected = node.getInput<std::string>("zone_type");
            if (!expected) {
                throw BT::RuntimeError(expected.error());
            }

            const std::string goal_zone_type =
                blackboard(node)->get<std::string>(kGoalZoneTypeKey);
            return goal_zone_type == expected.value() ? BT::NodeStatus::SUCCESS
                                                      : BT::NodeStatus::FAILURE;
        },
        {BT::InputPort<std::string>("zone_type")});

    factory.registerSimpleAction(
        "SetPreferredProfile",
        [](BT::TreeNode& node) {
            const auto profile = node.getInput<std::string>("profile");
            if (!profile) {
                throw BT::RuntimeError(profile.error());
            }
            return setPreferredProfile(node, profile.value());
        },
        {BT::InputPort<std::string>("profile")});

    factory.registerSimpleAction(
        "ResolveTransitionProfile",
        [](BT::TreeNode& node) {
            auto* board = blackboard(node);
            std::string transition_profile;

            const std::string start_zone_type =
                board->get<std::string>(kStartZoneTypeKey);
            const std::string goal_zone_type =
                board->get<std::string>(kGoalZoneTypeKey);
            auto state = runtimeState(node);
            if (start_zone_type == "ManeuveringZone" &&
                goal_zone_type == "TrackMainRoad" &&
                state->behavior_set->contains("maneuver_to_track_profile")) {
                transition_profile = "maneuver_to_track_profile";
            }

            board->set<std::string>(kTransitionProfileKey, transition_profile);
            return BT::NodeStatus::SUCCESS;
        });

    factory.registerSimpleCondition(
        "CanRetryWithRelaxed",
        [](BT::TreeNode& node) {
            const std::string preferred =
                blackboard(node)->get<std::string>(kPreferredProfileKey);
            auto state = runtimeState(node);
            if (preferred == kRelaxedProfile) {
                return BT::NodeStatus::FAILURE;
            }
            return state->behavior_set->contains(kRelaxedProfile)
                       ? BT::NodeStatus::SUCCESS
                       : BT::NodeStatus::FAILURE;
        });

    factory.registerSimpleAction(
        "RunPlanningAttempt",
        [](BT::TreeNode& node) {
            const auto profile = node.getInput<std::string>("profile");
            if (!profile) {
                throw BT::RuntimeError(profile.error());
            }

            auto state = runtimeState(node);
            if (!state->behavior_set->contains(profile.value())) {
                state->detail = "Planner behavior '" + profile.value() + "' is not defined.";
                state->plan_succeeded = false;
                blackboard(node)->set<bool>(kPlanSucceededKey, false);
                return BT::NodeStatus::FAILURE;
            }

            blackboard(node)->set<std::string>(kSelectedProfileKey, profile.value());
            blackboard(node)->set<size_t>(kAttemptIndexKey, state->attempt_index);
            state->attempted_profiles.push_back(profile.value());

            const PlannerRunResult result =
                state->runner({profile.value(),
                               blackboard(node)->get<std::string>(kTransitionProfileKey),
                               state->attempt_index});
            state->detail = result.detail;
            state->plan_succeeded = result.success;
            blackboard(node)->set<bool>(kPlanSucceededKey, result.success);

            ++state->attempt_index;
            return result.success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        },
        {BT::InputPort<std::string>("profile")});
}

} // namespace

BehaviorTreePlannerOrchestrator::BehaviorTreePlannerOrchestrator(
    std::string xml_filepath,
    PlannerBehaviorSet behavior_set)
    : xml_filepath_(std::move(xml_filepath)),
      behavior_set_(std::move(behavior_set)) {}

BehaviorTreePlanResult BehaviorTreePlannerOrchestrator::run(
    const PlanningRequestContext& request,
    const PlannerAttemptRunner& runner) const {
    if (!runner) {
        throw std::invalid_argument(
            "BehaviorTreePlannerOrchestrator requires a non-empty runner callback.");
    }

    auto blackboard = BT::Blackboard::create();
    blackboard->set<std::string>(kPlanningIntentKey, toString(request.intent));
    blackboard->set<std::string>(kStartZoneTypeKey, zoneTypeName(request.start_zone));
    blackboard->set<std::string>(kGoalZoneTypeKey, zoneTypeName(request.goal_zone));
    blackboard->set<std::string>(kStartZoneBehaviorKey, zoneBehaviorName(request.start_zone));
    blackboard->set<std::string>(kGoalZoneBehaviorKey, zoneBehaviorName(request.goal_zone));
    blackboard->set<std::string>(kPreferredProfileKey, "");
    blackboard->set<std::string>(kSelectedProfileKey, "");
    blackboard->set<std::string>(kTransitionProfileKey, "");
    blackboard->set<size_t>(kAttemptIndexKey, 0);
    blackboard->set<bool>(kPlanSucceededKey, false);

    auto state = std::make_shared<RuntimeState>();
    state->runner = runner;
    state->behavior_set = &behavior_set_;
    blackboard->set<std::shared_ptr<RuntimeState>>(kRuntimeStateKey, state);

    BT::BehaviorTreeFactory factory;
    registerPlannerNodes(factory);

    BT::Tree tree = factory.createTreeFromFile(xml_filepath_, blackboard);
    const BT::NodeStatus status = tree.tickWhileRunning();

    BehaviorTreePlanResult result;
    result.success = (status == BT::NodeStatus::SUCCESS) && state->plan_succeeded;
    result.preferred_profile = blackboard->get<std::string>(kPreferredProfileKey);
    result.selected_profile = blackboard->get<std::string>(kSelectedProfileKey);
    result.detail = state->detail;
    result.attempted_profiles = state->attempted_profiles;

    if (!result.success && result.detail.empty()) {
        if (result.preferred_profile.empty()) {
            result.detail = "Unable to resolve a valid planner behavior.";
        } else {
            result.detail = "Planning failed for all attempted behavior profiles.";
        }
    }

    return result;
}

std::string toString(PlanningIntent intent) {
    switch (intent) {
    case PlanningIntent::NORMAL:
        return "normal";
    case PlanningIntent::TIGHT_MANEUVER:
        return "tight_maneuver";
    }
    return "normal";
}

} // namespace planning
} // namespace coastmotionplanning
