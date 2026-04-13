#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "coastmotionplanning/planning/behavior_tree_planner_orchestrator.hpp"
#include "coastmotionplanning/planning/planner_behavior_set.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

namespace {

using coastmotionplanning::geometry::Polygon2d;
using coastmotionplanning::planning::BehaviorTreePlanResult;
using coastmotionplanning::planning::BehaviorTreePlannerOrchestrator;
using coastmotionplanning::planning::PlannerBehaviorSet;
using coastmotionplanning::planning::PlannerRunResult;
using coastmotionplanning::planning::PlanningAttempt;
using coastmotionplanning::planning::PlanningIntent;
using coastmotionplanning::planning::PlanningRequestContext;

Polygon2d makeSquarePolygon() {
    Polygon2d polygon;
    polygon.outer().push_back({0.0, 0.0});
    polygon.outer().push_back({10.0, 0.0});
    polygon.outer().push_back({10.0, 10.0});
    polygon.outer().push_back({0.0, 10.0});
    polygon.outer().push_back({0.0, 0.0});
    return polygon;
}

std::shared_ptr<coastmotionplanning::zones::ManeuveringZone> makeManeuveringZone(
    const std::string& behavior = "") {
    auto zone = std::make_shared<coastmotionplanning::zones::ManeuveringZone>(
        makeSquarePolygon(), "maneuvering");
    if (!behavior.empty()) {
        zone->setPlannerBehavior(behavior);
    }
    return zone;
}

std::shared_ptr<coastmotionplanning::zones::TrackMainRoad> makeTrackZone(
    const std::string& behavior = "") {
    auto zone = std::make_shared<coastmotionplanning::zones::TrackMainRoad>(
        makeSquarePolygon(), "track");
    if (!behavior.empty()) {
        zone->setPlannerBehavior(behavior);
    }
    return zone;
}

std::string repoPath(const std::string& relative_path) {
    return std::string(COAST_REPO_ROOT) + "/" + relative_path;
}

BehaviorTreePlannerOrchestrator makeOrchestrator() {
    return BehaviorTreePlannerOrchestrator(
        repoPath("configs/behavior_trees/planner_behavior_tree.xml"),
        PlannerBehaviorSet::loadFromFile(repoPath("configs/planner_behaviors.yaml")));
}

TEST(PlannerBehaviorSetTest, LoadsNamedBehaviorsFromConfig) {
    const PlannerBehaviorSet behavior_set =
        PlannerBehaviorSet::loadFromFile(repoPath("configs/planner_behaviors.yaml"));

    EXPECT_TRUE(behavior_set.contains("primary_profile"));
    EXPECT_TRUE(behavior_set.contains("relaxed_profile"));
    EXPECT_TRUE(behavior_set.contains("parking_profile"));
    EXPECT_FALSE(behavior_set.contains("missing_profile"));
}

TEST(BehaviorTreePlannerTest, GoalZoneHintWinsForNormalRequests) {
    auto orchestrator = makeOrchestrator();

    PlanningRequestContext request;
    request.intent = PlanningIntent::NORMAL;
    request.start_zone = makeManeuveringZone("parking_profile");
    request.goal_zone = makeTrackZone("primary_profile");

    const BehaviorTreePlanResult result = orchestrator.run(
        request,
        [](const PlanningAttempt& attempt) {
            return PlannerRunResult{
                attempt.profile == "primary_profile",
                attempt.profile == "primary_profile" ? "planned" : "unexpected profile"
            };
        });

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.preferred_profile, "primary_profile");
    EXPECT_EQ(result.selected_profile, "primary_profile");
    ASSERT_EQ(result.attempted_profiles.size(), 1u);
    EXPECT_EQ(result.attempted_profiles[0], "primary_profile");
}

TEST(BehaviorTreePlannerTest, TightManeuverRetriesWithRelaxedAfterParkingFailure) {
    auto orchestrator = makeOrchestrator();

    PlanningRequestContext request;
    request.intent = PlanningIntent::TIGHT_MANEUVER;
    request.start_zone = makeTrackZone("primary_profile");
    request.goal_zone = makeTrackZone("primary_profile");

    const BehaviorTreePlanResult result = orchestrator.run(
        request,
        [](const PlanningAttempt& attempt) {
            if (attempt.profile == "parking_profile") {
                return PlannerRunResult{false, "parking attempt failed"};
            }
            if (attempt.profile == "relaxed_profile") {
                return PlannerRunResult{true, "relaxed attempt succeeded"};
            }
            return PlannerRunResult{false, "unexpected profile"};
        });

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.preferred_profile, "parking_profile");
    EXPECT_EQ(result.selected_profile, "relaxed_profile");
    ASSERT_EQ(result.attempted_profiles.size(), 2u);
    EXPECT_EQ(result.attempted_profiles[0], "parking_profile");
    EXPECT_EQ(result.attempted_profiles[1], "relaxed_profile");
}

TEST(BehaviorTreePlannerTest, StartZoneHintIsUsedWhenGoalHintIsMissing) {
    auto orchestrator = makeOrchestrator();

    PlanningRequestContext request;
    request.intent = PlanningIntent::NORMAL;
    request.start_zone = makeManeuveringZone("parking_profile");
    request.goal_zone = makeTrackZone();

    const BehaviorTreePlanResult result = orchestrator.run(
        request,
        [](const PlanningAttempt& attempt) {
            return PlannerRunResult{
                attempt.profile == "parking_profile",
                attempt.profile == "parking_profile" ? "planned" : "unexpected profile"
            };
        });

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.preferred_profile, "parking_profile");
    EXPECT_EQ(result.selected_profile, "parking_profile");
    ASSERT_EQ(result.attempted_profiles.size(), 1u);
    EXPECT_EQ(result.attempted_profiles[0], "parking_profile");
}

TEST(BehaviorTreePlannerTest, RelaxedProfileDoesNotRetryAgain) {
    auto orchestrator = makeOrchestrator();

    PlanningRequestContext request;
    request.intent = PlanningIntent::NORMAL;
    request.start_zone = makeManeuveringZone();
    request.goal_zone = makeManeuveringZone("relaxed_profile");

    const BehaviorTreePlanResult result = orchestrator.run(
        request,
        [](const PlanningAttempt& attempt) {
            return PlannerRunResult{
                false,
                "attempt failed for " + attempt.profile
            };
        });

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.preferred_profile, "relaxed_profile");
    EXPECT_EQ(result.selected_profile, "relaxed_profile");
    ASSERT_EQ(result.attempted_profiles.size(), 1u);
    EXPECT_EQ(result.attempted_profiles[0], "relaxed_profile");
}

TEST(BehaviorTreePlannerTest, InvalidGoalZoneHintFallsBackToZoneTypeDefault) {
    auto orchestrator = makeOrchestrator();

    PlanningRequestContext request;
    request.intent = PlanningIntent::NORMAL;
    request.start_zone = makeManeuveringZone();
    request.goal_zone = makeTrackZone("not_in_catalog");

    const BehaviorTreePlanResult result = orchestrator.run(
        request,
        [](const PlanningAttempt& attempt) {
            return PlannerRunResult{
                attempt.profile == "primary_profile",
                attempt.profile == "primary_profile" ? "planned" : "unexpected profile"
            };
        });

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.preferred_profile, "primary_profile");
    EXPECT_EQ(result.selected_profile, "primary_profile");
    ASSERT_EQ(result.attempted_profiles.size(), 1u);
    EXPECT_EQ(result.attempted_profiles[0], "primary_profile");
}

} // namespace
