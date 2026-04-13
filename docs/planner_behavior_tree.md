# Planner Behavior Tree

This repository now includes a BehaviorTree.CPP orchestration seam for selecting planner behavior profiles at runtime.

## Runtime Inputs

The orchestrator blackboard uses these keys:

- `planning_intent`: `normal` or `tight_maneuver`
- `start_zone_type`
- `goal_zone_type`
- `start_zone_behavior`
- `goal_zone_behavior`
- `preferred_profile`
- `selected_profile`
- `attempt_index`
- `plan_succeeded`

## Resolution Order

The XML tree in `configs/behavior_trees/planner_behavior_tree.xml` resolves the preferred profile in this order:

1. `tight_maneuver` requests prefer `parking_profile`
2. Goal-zone `planner_behavior`
3. Start-zone `planner_behavior`
4. `TrackMainRoad` goals default to `primary_profile`
5. `ManeuveringZone` goals default to `parking_profile`
6. Fallback default is `primary_profile`

## Retry Policy

The main tree attempts planning once with the resolved preferred profile. If that attempt fails and the preferred profile is not already `relaxed_profile`, it retries once with `relaxed_profile`.

## Current Integration Scope

The orchestrator is intentionally separate from costmap construction and any concrete Hybrid A* runner, because this repository still does not contain a planner execution pipeline. Callers are expected to provide the request context, load the planner behavior set from `configs/planner_behaviors.yaml`, and inject a planner-attempt callback.
