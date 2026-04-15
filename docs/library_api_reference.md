# CoastMotionPlanning Library API Reference

This document describes the public API exposed by the headers in `include/coastmotionplanning` and by the CMake targets defined in this repository. It is intended for application developers who want to embed CoastMotionPlanning in another C++ app.

The reference is based on the current public headers and the runtime behavior wired through `src/`. Internal-only implementation types are intentionally omitted.

## 1. How to consume the library

This repository currently defines two static-library targets:

- `coast_motion_planning`
- `coast_motion_planning_visualizer`

There are no `install()` or exported package rules yet, so the supported integration model today is:

1. add the repo as a subdirectory or `FetchContent` dependency
2. link one of the library targets directly
3. include headers from `include/coastmotionplanning/...`

Example:

```cmake
include(FetchContent)

FetchContent_Declare(
  coast_motion_planning
  GIT_REPOSITORY <your-fork-or-local-mirror>
  GIT_TAG main
)
FetchContent_MakeAvailable(coast_motion_planning)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE coast_motion_planning)
```

If you want the local browser visualizer service as well:

```cmake
target_link_libraries(my_app PRIVATE coast_motion_planning_visualizer)
```

## 2. Public targets

| Target | Purpose | Main dependencies exposed transitively |
| --- | --- | --- |
| `coast_motion_planning` | Core planning library: robot models, map parsing, zones, costmaps, heuristics, collision checking, planner behavior profiles, behavior-tree orchestration, Hybrid A*. | Boost headers, `yaml-cpp`, `grid_map_core`, `ompl` |
| `coast_motion_planning_visualizer` | Embeddable HTTP service and DTO layer for the planner visualizer. Links the core library plus JSON/HTTP server support. | `coast_motion_planning`, `httplib`, `nlohmann_json` |

## 3. End-to-end planning flow

The lowest-friction planning flow for another app is:

1. Load a `config::CarDefinition` from `robots.yaml`.
2. Parse zones from a map YAML with `map::MapParser`.
3. Load planner profiles with `planning::PlannerBehaviorSet::loadFromFile()`.
4. Override motion-primitive constraints from the selected robot.
5. Construct `planning::HybridAStarPlanner`.
6. Build a `planning::HybridAStarPlannerRequest`.
7. Call `plan()`.

Minimal example:

```cpp
#include "coastmotionplanning/config/robots_parser.hpp"
#include "coastmotionplanning/map/map_parser.hpp"
#include "coastmotionplanning/math/angle.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/planning/hybrid_a_star_planner.hpp"
#include "coastmotionplanning/planning/planner_behavior_set.hpp"

using namespace coastmotionplanning;

int main() {
  const auto car_def = config::RobotsParser::loadCarDefinition(
      "configs/robots.yaml", "Pro_XD");
  auto zones = map::MapParser::parse("configs/maps/largo_map.yaml");

  auto behaviors =
      planning::PlannerBehaviorSet::loadFromFile("configs/planner_behaviors.yaml");
  behaviors.overrideMotionPrimitiveConstraints(
      car_def.minTurningRadiusMeters(),
      car_def.maxSteerAngleRadians());

  robot::Car car = car_def.buildCar();
  planning::HybridAStarPlanner planner(car, zones, behaviors);

  planning::HybridAStarPlannerRequest request;
  request.start = math::Pose2d{0.0, 0.0, math::Angle::from_degrees(0.0)};
  request.goal = math::Pose2d{20.0, 2.0, math::Angle::from_degrees(0.0)};
  request.initial_behavior_name = "primary_profile";
  request.transition_behavior_name = "maneuver_to_track_profile";
  request.dual_model_lut_path = "configs/heuristics/nh_lut_dual.bin"; // optional

  const auto result = planner.plan(request);
  if (!result.success) {
    throw std::runtime_error(result.detail);
  }

  return 0;
}
```

Important runtime conventions:

- `PlannerBehaviorSet::loadFromFile(".../planner_behaviors.yaml")` automatically expects a sibling file named `master_params.yaml` in the same directory.
- `HybridAStarPlanner` is currently car-only. There is no `TruckTrailer` overload yet.
- `request.dual_model_lut_path` is optional. If no LUT is loaded, the planner still configures OMPL-based heuristic spaces at runtime.
- `HybridAStarPlannerResult::debug_trace` is only populated when `global.debug_mode: true` in `planner_behaviors.yaml`.

## 4. Public API by namespace

### `coastmotionplanning::common`

#### `common/types.hpp`

- `enum class MotionDirection { Forward, Reverse };`
  Used in planner outputs to describe each path segment's drive direction.

#### `common/math_constants.hpp`

- `PI`
- `HALF_PI`
- `TWO_PI`
- `DEG_TO_RAD`
- `RAD_TO_DEG`

#### `common/profiling.hpp`

- `struct ProfilingScopeSummary`
  - `std::string scope_name`
  - `uint64_t count`
  - `double total_ms`
  - `double avg_ms`
  - `double max_ms`

- `class ProfilingCollector`
  - `void record(const std::string& scope_name, double elapsed_ms)`
  - `std::vector<ProfilingScopeSummary> snapshotSortedByTotalMs() const`
  - `void mergeFrom(const ProfilingCollector& other)`

- `class ScopedProfilingTimer`
  - `ScopedProfilingTimer(ProfilingCollector* collector, std::string scope_name)`
  - destructor records elapsed time if `collector != nullptr`

This module is useful if your app wants timing summaries from costmap building or planner debug traces.

### `coastmotionplanning::geometry`

#### `geometry/shape_types.hpp`

Boost.Geometry aliases:

- `using Point2d = bg::model::d2::point_xy<double>;`
- `using Polygon2d = bg::model::polygon<Point2d>;`
- `using Box2d = bg::model::box<Point2d>;`
- `using LineString2d = bg::model::linestring<Point2d>;`

These types are used throughout zones, robot footprints, and geometry utilities.

### `coastmotionplanning::math`

#### `math/angle.hpp`

- `class Angle`
  - `static Angle from_radians(double rad)`
  - `static Angle from_degrees(double deg)`
  - `double radians() const`
  - `double degrees() const`
  - `void normalize()`
  - `Angle normalized() const`
  - arithmetic operators `+`, `-`, `+=`, `-=`, `*`, `/`
  - equality operator `==`

`Angle` stores radians internally and normalizes to `[-pi, pi)` when requested.

#### `math/pose2d.hpp`

- `struct Pose2d`
  - `double x`
  - `double y`
  - `Angle theta`
  - default constructor
  - `Pose2d(double x_, double y_, Angle theta_)`
  - `bool operator==(const Pose2d& other) const`

This is the standard world-frame pose type used by maps, zones, planners, and results.

### `coastmotionplanning::robot`

#### `robot/robot_base.hpp`

- `struct RobotState`
  - `double x`
  - `double y`
  - `double yaw`
  - `std::vector<double> articulations`

- `class RobotBase`
  - `RobotBase(double max_width, double max_length)`
  - virtual destructor
  - `virtual double getMaxWidth() const`
  - `virtual double getMaxLength() const`
  - `virtual geometry::Polygon2d getRobotFootprint(const RobotState& state) const = 0`

`RobotState::articulations` is where articulated models can store hitch angle or similar joints.

#### `robot/car.hpp`

- `class Car : public RobotBase`
  - `Car(double width, double wheelbase, double front_overhang, double rear_overhang)`
  - `geometry::Polygon2d getRobotFootprint(const RobotState& state) const override`
  - `double getWidth() const`
  - `double getWheelbase() const`
  - `double getFrontOverhang() const`
  - `double getRearOverhang() const`

This is the only robot model currently wired into the end-to-end planner path.

#### `robot/truck_trailer.hpp`

- `struct TractorParams`
  - `double width`
  - `double wheelbase`
  - `double front_overhang`
  - `double rear_overhang`
  - `double hitch_offset`

- `struct TrailerParams`
  - `double width`
  - `double wheelbase`
  - `double front_overhang`
  - `double rear_overhang`

- `class TruckTrailer : public RobotBase`
  - nested `struct Params`
    - `double tractor_width`
    - `double tractor_wheelbase`
    - `double tractor_front_overhang`
    - `double tractor_rear_overhang`
    - `double hitch_offset`
    - `double trailer_width`
    - `double trailer_wheelbase`
    - `double trailer_front_overhang`
    - `double trailer_rear_overhang`
  - `explicit TruckTrailer(const Params& params)`
  - `geometry::Polygon2d getRobotFootprint(const RobotState& state) const override`
  - `const Params& getParams() const`

This class exists as a reusable footprint model, but there is no public parser helper for `TruckTrailer::Params` and no `HybridAStarPlanner` overload for truck-trailer planning yet.

### `coastmotionplanning::config`

#### `config/robots_parser.hpp`

- `struct RobotDefinition`
  - `std::string name`
  - `std::string type`

- `struct CarDefinition`
  - `std::string name`
  - `double width_m`
  - `double wheelbase_m`
  - `double front_overhang_m`
  - `double rear_overhang_m`
  - `double max_steer_angle_deg`
  - `double max_speed_mps`
  - `double max_reverse_speed_mps`
  - `robot::Car buildCar() const`
  - `double maxSteerAngleRadians() const`
  - `double minTurningRadiusMeters() const`

- `class RobotsParser`
  - `static std::vector<RobotDefinition> parse(const std::string& filepath)`
  - `static std::vector<CarDefinition> parseCarDefinitions(const std::string& filepath)`
  - `static CarDefinition loadCarDefinition(const std::string& filepath, const std::string& robot_name)`
  - `static robot::Car loadCar(const std::string& filepath, const std::string& robot_name)`

Behavior notes:

- `parse()` validates both `Car` and `TruckTrailer` entries.
- `parseCarDefinitions()`, `loadCarDefinition()`, and `loadCar()` only return `Car` entries.
- robot names must be unique and must use `name`, not deprecated `id`.

#### `config/planner_behavior_parser.hpp`

- `using PlannerBehaviorProfiles = std::unordered_map<std::string, planning::PlannerBehaviorProfile>;`

- `struct PlannerBehaviorConfigFile`
  - `planning::PlannerBehaviorGlobalConfig global`
  - `PlannerBehaviorProfiles profiles`

- `class PlannerBehaviorParser`
  - `static PlannerBehaviorConfigFile parse(const std::string& master_params_filepath, const std::string& behaviors_filepath)`

Behavior notes:

- `parse()` validates that every behavior profile repeats the full schema from `master_params.yaml`.
- unexpected keys cause an exception.
- `active_layers` is required for every behavior profile.

### `coastmotionplanning::zones`

#### `zones/zone.hpp`

- `class Zone`
  - default constructor
  - virtual destructor
  - `Zone(const geometry::Polygon2d& polygon, const std::optional<std::string>& name = std::nullopt)`
  - `const geometry::Polygon2d& getPolygon() const`
  - `void setPolygon(const geometry::Polygon2d& polygon)`
  - `const std::optional<std::string>& getName() const`
  - `void setName(const std::optional<std::string>& name)`
  - `const std::optional<std::string>& getPlannerBehavior() const`
  - `void setPlannerBehavior(const std::optional<std::string>& planner_behavior)`
  - `bool hasExplicitPlannerBehavior() const`
  - `std::string getResolvedPlannerBehavior() const`
  - `virtual std::string getDefaultPlannerBehavior() const = 0`
  - `virtual std::vector<std::string> getActiveLayers() const = 0`
  - `virtual bool isReverseAllowed() const = 0`

`getResolvedPlannerBehavior()` returns the explicitly assigned planner behavior if present, otherwise the zone type's default behavior.

#### `zones/maneuvering_zone.hpp`

- `class ManeuveringZone : public Zone`
  - default constructor
  - `ManeuveringZone(const geometry::Polygon2d& polygon, const std::optional<std::string>& name = std::nullopt)`
  - `std::string getDefaultPlannerBehavior() const override`
  - `std::vector<std::string> getActiveLayers() const override`
  - `bool isReverseAllowed() const override`

Current semantics:

- default planner behavior is an empty string
- active layers:
  - `static_obstacles`
  - `inflation`
  - `holonomic_with_obstacles`
- reverse is allowed unless the selected planner profile says otherwise

#### `zones/track_main_road.hpp`

- `class TrackMainRoad : public Zone`
  - default constructor
  - `TrackMainRoad(const geometry::Polygon2d& polygon, const std::optional<std::string>& name = std::nullopt)`
  - `TrackMainRoad(const geometry::Polygon2d& polygon, const std::vector<std::vector<geometry::Point2d>>& lane_point_sequences, const std::optional<std::string>& name = std::nullopt)`
  - `const std::vector<std::vector<math::Pose2d>>& getLanes() const`
  - `void setLanes(const std::vector<std::vector<math::Pose2d>>& lanes)`
  - `void addLaneFromPoints(const std::vector<geometry::Point2d>& points)`
  - `std::string getDefaultPlannerBehavior() const override`
  - `std::vector<std::string> getActiveLayers() const override`
  - `bool isReverseAllowed() const override`

Current semantics:

- default planner behavior is an empty string
- active layers:
  - `static_obstacles`
  - `inflation`
  - `lane_centerline_cost`
  - `holonomic_with_obstacles`
- reverse is allowed unless the selected planner profile says otherwise
- the zone requires exactly two valid, opposite-direction lanes

### `coastmotionplanning::map`

#### `map/coordinate_transform.hpp`

- `class CoordinateTransform`
  - nested `struct LLA`
    - `double lat`
    - `double lon`
    - `double alt`
  - `static Eigen::Vector3d llaToEcef(const LLA& lla)`
  - `static Eigen::Vector3d ecefToEnu(const Eigen::Vector3d& ecef, const LLA& origin_lla)`
  - `static Eigen::Vector3d llaToWorld(const LLA& lla, const LLA& origin_lla, double model_metric)`

#### `map/map_parser.hpp`

- `class MapParser`
  - `enum class CoordinateType { WORLD, LAT_LONG, LONG_LAT }`
  - `static std::vector<std::shared_ptr<zones::Zone>> parse(const std::string& filepath)`
  - `static std::vector<std::shared_ptr<zones::Zone>> parseFromString(const std::string& yaml_content, const std::string& source_label = "<memory>")`

Behavior notes:

- supports both file-based and in-memory YAML parsing
- throws on invalid YAML, duplicate zone names, deprecated `id`, missing geographic metadata, or invalid lane configuration
- zone YAML supports world coordinates and geographic coordinates

### `coastmotionplanning::motion_primitives`

#### `motion_primitives/motion_primitive_types.hpp`

- `enum class TurnDirection : uint8_t`
  - `FORWARD`
  - `LEFT`
  - `RIGHT`
  - `REVERSE`
  - `REV_LEFT`
  - `REV_RIGHT`
  - `UNKNOWN`

- `struct MotionPose`
  - `float x`
  - `float y`
  - `float theta`
  - `TurnDirection turn_dir`
  - constructors

- `using MotionPoses = std::vector<MotionPose>;`

- `struct TrigValues`
  - `double cos_val`
  - `double sin_val`

- `struct MotionTableConfig`
  - `float minimum_turning_radius`
  - `unsigned int num_angle_quantization`
  - `unsigned int size_x`
  - `float non_straight_penalty`
  - `float change_penalty`
  - `float reverse_penalty`
  - `float cost_penalty`
  - `float retrospective_penalty`
  - `bool use_quadratic_cost_penalty`
  - `bool allow_primitive_interpolation`

- `struct MotionPrimitive`
  - `unsigned int trajectory_id`
  - `float start_angle`
  - `float end_angle`
  - `float turning_radius`
  - `float trajectory_length`
  - `float arc_length`
  - `float straight_length`
  - `bool left_turn`
  - `MotionPoses poses`

- `using MotionPrimitives = std::vector<MotionPrimitive>;`

Important unit convention:

- motion-table coordinates are in map-cell units, not meters
- `theta` in primitive tables is stored in heading-bin increments, not radians

#### `motion_primitives/car_motion_table.hpp`

- `class CarMotionTable`
  - `enum class Model { DUBIN, REEDS_SHEPP }`
  - default constructor
  - `void initDubin(const MotionTableConfig& config)`
  - `void initReedsShepp(const MotionTableConfig& config)`
  - `MotionPoses getProjections(float node_x, float node_y, unsigned int heading_bin) const`
  - `unsigned int getClosestAngularBin(double theta_rad) const`
  - `float getAngleFromBin(unsigned int bin_idx) const`
  - `float getTravelCost(unsigned int primitive_idx) const`
  - accessors:
    - `getNumPrimitives()`
    - `getNumAngleBins()`
    - `getBinSize()`
    - `getMinTurningRadius()`
    - `getModel()`
    - `getNonStraightPenalty()`
    - `getChangePenalty()`
    - `getReversePenalty()`
    - `getCostPenalty()`
    - `getTravelDistanceReward()`
    - `getUseQuadraticCostPenalty()`

This is the main primitive source for the car planner.

#### `motion_primitives/truck_trailer_motion_table.hpp`

- `struct TruckTrailerPose`
  - `float x`
  - `float y`
  - `float theta1`
  - `float theta2`
  - `TurnDirection turn_dir`

- `using TruckTrailerPoses = std::vector<TruckTrailerPose>;`

- `struct TruckTrailerKinematics`
  - `float tractor_wheelbase`
  - `float hitch_offset`
  - `float trailer_wheelbase`
  - `float max_hitch_angle`

- `class TruckTrailerMotionTable`
  - default constructor
  - `void init(const MotionTableConfig& config, const TruckTrailerKinematics& kinematics)`
  - `TruckTrailerPoses getProjections(float node_x, float node_y, unsigned int tractor_heading_bin, unsigned int trailer_heading_bin) const`
  - `float getTravelCost(unsigned int primitive_idx) const`
  - `unsigned int getClosestAngularBin(double theta_rad) const`
  - `float getAngleFromBin(unsigned int bin_idx) const`
  - accessors:
    - `getNumPrimitives()`
    - `getNumAngleBins()`
    - `getBinSize()`
    - `getMinTurningRadius()`
    - `getMaxHitchAngle()`
    - `getNonStraightPenalty()`
    - `getChangePenalty()`
    - `getReversePenalty()`
    - `getCostPenalty()`
    - `getTravelDistanceReward()`
    - `getUseQuadraticCostPenalty()`

This is a lower-level building block for articulated motion planning. It is not currently consumed by `HybridAStarPlanner`.

### `coastmotionplanning::costs`

#### `costs/costmap_types.hpp`

- `struct CostValues`
  - `FREE_SPACE`
  - `INSCRIBED`
  - `LETHAL`
  - `NO_INFORMATION`

- `namespace CostmapLayerNames`
  - `STATIC_OBSTACLES`
  - `INFLATION`
  - `ZONE_CONSTRAINTS`
  - `LANE_CENTERLINE_COST`
  - `LANE_HEADING`
  - `LANE_DISTANCE`
  - `HOLONOMIC_WITH_OBSTACLES`
  - `COMBINED_COST`

- `struct ZoneConstraintValues`
  - `ZONE_NONE`
  - `ZONE_TRANSITION`

- `struct CostmapConfig`
  - `double resolution`
  - `double inflation_radius_m`
  - `double inscribed_radius_m`
  - `double cost_scaling_factor`
  - `double alpha_shape_alpha`
  - `double max_lane_cost`
  - `double max_lane_half_width`
  - `double hitch_angle_penalty_factor`

#### `costs/static_obstacle_layer.hpp`

- `class StaticObstacleLayer`
  - `static void build(grid_map::GridMap& costmap, const geometry::Polygon2d& search_boundary)`

#### `costs/inflation_layer.hpp`

- `class InflationLayer`
  - `static void build(grid_map::GridMap& costmap, double inflation_radius_m, double inscribed_radius_m, double cost_scaling_factor)`

#### `costs/zone_constraints_layer.hpp`

- `class ZoneConstraintsLayer`
  - `static void build(grid_map::GridMap& costmap, const ZoneSelectionResult& selection)`

#### `costs/lane_centerline_layer.hpp`

- `class LaneCenterlineLayer`
  - `static void build(grid_map::GridMap& costmap, const std::vector<std::shared_ptr<zones::Zone>>& selected_zones, double max_lane_cost, double max_half_width)`

#### `costs/holonomic_obstacles_heuristic.hpp`

- `class HolonomicObstaclesHeuristic`
  - `static void compute(grid_map::GridMap& costmap, const grid_map::Position& goal_position)`

#### `costs/non_holonomic_heuristic.hpp`

- `struct NHLutHeader`
  - `MAGIC`
  - `VERSION`
  - `uint32_t magic`
  - `uint32_t version`
  - `float min_turning_radius`
  - `uint32_t num_angle_bins`
  - `uint32_t grid_size`
  - `float cell_size`
  - `uint32_t stored_y_size`
  - `uint32_t reserved`

- `class NonHolonomicHeuristic`
  - default constructor
  - `bool loadFromFile(const std::string& filepath)`
  - `bool saveToFile(const std::string& filepath) const`
  - `float lookup(float dx, float dy, float dtheta) const`
  - `float lookupWithHitchPenalty(float dx, float dy, float dtheta_tractor, float hitch_angle, float goal_hitch, float penalty_factor) const`
  - `void initGrid(float min_turning_radius, uint32_t num_angle_bins, uint32_t grid_size, float cell_size)`
  - `float& at(int x_idx, int y_idx, int theta_idx)`
  - `float at(int x_idx, int y_idx, int theta_idx) const`
  - accessors:
    - `getMinTurningRadius()`
    - `getNumAngleBins()`
    - `getGridSize()`
    - `getCellSize()`
    - `isLoaded()`

`CostmapBuilder` exposes this single-model LUT loader, but the main planner currently uses `DualModelNonHolonomicHeuristic`.

#### `costs/dual_model_non_holonomic_heuristic.hpp`

- `enum class HeuristicModel : uint8_t`
  - `DUBINS`
  - `REEDS_SHEPP`

- `struct DualModelNHLutHeader`
  - `MAGIC`
  - `VERSION`
  - `uint32_t magic`
  - `uint32_t version`
  - `float min_turning_radius`
  - `uint32_t num_angle_bins`
  - `uint32_t grid_size`
  - `float cell_size`
  - `uint32_t stored_y_size`
  - `uint32_t model_count`

- `class DualModelNonHolonomicHeuristic`
  - constructor/destructor
  - move constructor and move assignment
  - copy operations deleted
  - `void configureOmpl(float min_turning_radius)`
  - `void initGrid(float min_turning_radius, uint32_t num_angle_bins, uint32_t grid_size, float cell_size)`
  - `bool loadFromFile(const std::string& filepath)`
  - `bool saveToFile(const std::string& filepath) const`
  - `float lookup(HeuristicModel model, float dx, float dy, float dtheta) const`
  - `float& at(HeuristicModel model, int x_idx, int y_idx, int theta_idx)`
  - `float at(HeuristicModel model, int x_idx, int y_idx, int theta_idx) const`
  - `std::pair<double, std::vector<std::array<double, 3>>> samplePath(HeuristicModel model, double from_x, double from_y, double from_yaw, double to_x, double to_y, double to_yaw, double step_size_m) const`
  - accessors:
    - `getMinTurningRadius()`
    - `getNumAngleBins()`
    - `getGridSize()`
    - `getCellSize()`
    - `hasLookupTable()`
    - `hasOmplSpaces()`

Behavior notes:

- if a LUT is loaded and the query is in bounds, `lookup()` uses the table
- otherwise it falls back to OMPL distance computation if OMPL spaces are configured
- `samplePath()` is used by the planner for analytic expansions

#### `costs/zone_selector.hpp`

- `enum class SearchFrontierRole`
  - `StartZone`
  - `Transition`
  - `GoalZone`

- `struct SearchFrontierDescriptor`
  - `size_t frontier_id`
  - `SearchFrontierRole role`
  - `std::shared_ptr<zones::Zone> zone`
  - `std::string behavior_name`
  - `std::optional<size_t> next_frontier_id`

- `struct ZoneSelectionResult`
  - `std::vector<std::shared_ptr<zones::Zone>> selected_zones`
  - `std::vector<SearchFrontierDescriptor> frontiers`
  - `geometry::Polygon2d search_boundary`

- `class ZoneSelector`
  - default constructor
  - `ZoneSelectionResult select(const math::Pose2d& start, const math::Pose2d& goal, const std::vector<std::shared_ptr<zones::Zone>>& all_zones, double alpha = 0.0, const std::string& start_frontier_behavior = "", const std::string& transition_frontier_behavior = "") const`
  - `static std::shared_ptr<zones::Zone> findContainingZone(const geometry::Point2d& point, const std::vector<std::shared_ptr<zones::Zone>>& zones)`
  - `static geometry::Polygon2d computeConcaveHull(const std::vector<geometry::Polygon2d>& zone_polygons, double alpha)`
  - `static bool isInsidePolygon(const geometry::Point2d& point, const geometry::Polygon2d& polygon)`

#### `costs/costmap_builder.hpp`

- `class CostmapBuilder`
  - `CostmapBuilder(const CostmapConfig& config, const std::vector<std::shared_ptr<zones::Zone>>& all_zones, const robot::RobotBase& robot, common::ProfilingCollector* profiler = nullptr)`
  - `grid_map::GridMap build(const math::Pose2d& start, const math::Pose2d& goal)`
  - `grid_map::GridMap build(const ZoneSelectionResult& selection, const math::Pose2d& goal)`
  - `bool loadNonHolonomicHeuristic(const std::string& filepath)`
  - `const NonHolonomicHeuristic& getNonHolonomicHeuristic() const`
  - `const grid_map::GridMap& getCostmap() const`

Current builder behavior:

- it always creates these standard layers:
  - `static_obstacles`
  - `inflation`
  - `zone_constraints`
  - `lane_centerline_cost`
  - `lane_heading`
  - `lane_distance`
  - `holonomic_with_obstacles`
  - `combined_cost`
- `PlannerBehaviorProfile::active_layers` affects planner cost usage, not which layers the builder generates

### `coastmotionplanning::collision_checking`

#### `collision_checking/collision_checker.hpp`

- `struct CollisionResult`
  - `bool in_collision`
  - `float max_cost`
  - `grid_map::Position worst_cell`

- `struct CollisionCheckerConfig`
  - `std::string obstacle_layer`
  - `float lethal_threshold`
  - `float inscribed_threshold`

- `class CollisionChecker`
  - `explicit CollisionChecker(const CollisionCheckerConfig& config = {})`
  - `CollisionResult checkCollision(const grid_map::GridMap& costmap, const robot::RobotBase& robot, const robot::RobotState& state) const`
  - `CollisionResult checkFootprint(const grid_map::GridMap& costmap, const geometry::Polygon2d& footprint) const`
  - `float getFootprintCost(const grid_map::GridMap& costmap, const geometry::Polygon2d& footprint) const`
  - `bool isPointInCollision(const grid_map::GridMap& costmap, const grid_map::Position& position) const`
  - `const CollisionCheckerConfig& getConfig() const`
  - `void setConfig(const CollisionCheckerConfig& config)`

The planner uses this class against the `combined_cost` layer for start, goal, primitive, and analytic-path validation.

### `coastmotionplanning::planning`

#### `planning/planner_behavior_profile.hpp`

- `struct PlannerBehaviorGlobalConfig`
  - `bool debug_mode`

- `struct PlannerBehaviorPlannerConfig`
  - `int max_planning_time_ms`
  - `double xy_grid_resolution_m`
  - `double yaw_grid_resolution_deg`
  - `double step_size_m`
  - `bool only_forward_path`
  - `double weight_forward`
  - `double weight_reverse`
  - `double weight_steer`
  - `double weight_steer_change`
  - `double weight_gear_change`
  - `double analytic_expansion_max_length_m`
  - `double analytic_expansion_ratio`
  - `double min_path_len_in_same_motion`
  - `bool analytic_shot`
  - `bool near_goal_analytic_expansion`
  - `double near_goal_analytic_radius_m`
  - `double weight_lane_centerline`
  - `double lane_heading_bias_weight`
  - `double max_cross_track_error_m`
  - `bool lane_primitive_suppression`

- `struct PlannerBehaviorCostmapConfig`
  - `double resolution_m`
  - `double inflation_radius_m`
  - `double inscribed_radius_m`
  - `double cost_scaling_factor`
  - `double alpha_shape_alpha`
  - `double max_lane_cost`
  - `double max_lane_half_width_m`
  - `costs::CostmapConfig toCostmapConfig() const`

- `struct PlannerBehaviorCollisionCheckerConfig`
  - `std::string collision_mode`
  - `double lethal_threshold`
  - `double margin_m`

- `struct PlannerBehaviorMotionPrimitivesConfig`
  - `int num_angle_bins`
  - `double min_turning_radius_m`
  - `double max_steer_angle_rad`

- `struct PlannerBehaviorNonHolonomicHeuristicConfig`
  - `int lut_grid_size`
  - `double lut_cell_size_m`
  - `double hitch_angle_penalty_factor`

- `struct PlannerBehaviorProfile`
  - `PlannerBehaviorPlannerConfig planner`
  - `PlannerBehaviorCostmapConfig costmap`
  - `PlannerBehaviorCollisionCheckerConfig collision_checker`
  - `PlannerBehaviorMotionPrimitivesConfig motion_primitives`
  - `PlannerBehaviorNonHolonomicHeuristicConfig non_holonomic_heuristic`
  - `std::unordered_set<std::string> active_layers`
  - `bool isLayerActive(const std::string& layer_name) const`
  - `costs::CostmapConfig makeCostmapConfig() const`

#### `planning/planner_behavior_set.hpp`

- `class PlannerBehaviorSet`
  - default constructor
  - `static PlannerBehaviorSet loadFromFile(const std::string& filepath)`
  - `bool contains(const std::string& behavior_name) const`
  - `const PlannerBehaviorProfile& get(const std::string& behavior_name) const`
  - `const PlannerBehaviorGlobalConfig& globalConfig() const`
  - `bool debugModeEnabled() const`
  - `void overrideMotionPrimitiveConstraints(double min_turning_radius_m, double max_steer_angle_rad)`
  - `void setMinimumPlanningTimeMs(int minimum_planning_time_ms)`
  - `const std::vector<std::string>& names() const`

`overrideMotionPrimitiveConstraints()` must usually be called after selecting a robot, because the planner profiles do not hardcode turning radius and steer angle in `planner_behaviors.yaml`.

#### `planning/planner_behavior_resolver.hpp`

- `struct ResolvedPlannerBehavior`
  - `size_t frontier_id`
  - `std::shared_ptr<zones::Zone> zone`
  - `std::string behavior_name`
  - `const PlannerBehaviorProfile* profile`
  - `bool switched_zone`
  - `bool switched_frontier`

- `class PlannerBehaviorResolver`
  - `static ResolvedPlannerBehavior resolve(const math::Pose2d& successor_pose, const grid_map::GridMap& costmap, size_t current_frontier_id, const std::shared_ptr<zones::Zone>& current_zone, const std::string& current_behavior_name, const std::vector<costs::SearchFrontierDescriptor>& frontiers, const PlannerBehaviorSet& behavior_set)`

This is the low-level mechanism used by the planner to decide whether a successor moved into a new zone/frontier and therefore a new behavior profile.

#### `planning/hybrid_a_star_planner.hpp`

Debug and result structs:

- `PlannerPrimitiveDebugEvent`
- `PlannerAnalyticDebugEvent`
- `PlannerExpansionDebugEvent`
- `PlannerFrontierDebugSummary`
- `PlannerFrontierHandoffDebugSummary`
- `HybridAStarPlannerDebugTrace`

These structs expose counters, timings, frontier summaries, analytic-expansion details, and primitive-level outcomes when debug mode is enabled.

Main request/result types:

- `struct HybridAStarPlannerRequest`
  - `math::Pose2d start`
  - `math::Pose2d goal`
  - `std::string initial_behavior_name`
  - `std::string transition_behavior_name`
  - `std::string dual_model_lut_path`

- `struct HybridAStarPlannerResult`
  - `bool success`
  - `std::string detail`
  - `std::vector<math::Pose2d> poses`
  - `std::vector<std::string> behavior_sequence`
  - `std::vector<common::MotionDirection> segment_directions`
  - `std::shared_ptr<HybridAStarPlannerDebugTrace> debug_trace`

Main class:

- `class HybridAStarPlanner`
  - `HybridAStarPlanner(const robot::Car& car, std::vector<std::shared_ptr<zones::Zone>> all_zones, PlannerBehaviorSet behavior_set)`
  - `HybridAStarPlannerResult plan(const HybridAStarPlannerRequest& request) const`
  - `static costs::HeuristicModel selectHeuristicModel(const ResolvedPlannerBehavior& resolved_behavior)`
  - `static bool isPrimitiveAllowed(motion_primitives::TurnDirection turn_direction, const ResolvedPlannerBehavior& resolved_behavior)`

Behavior notes:

- fails fast if the initial or transition behavior name is not defined
- fails fast if robot-derived motion-primitive constraints were never injected into the behavior set
- validates that start and goal are inside selected zones and not in collision
- uses `ZoneSelector` and `PlannerBehaviorResolver` internally to switch profiles across frontiers
- returns `behavior_sequence` alongside poses so your app can see profile transitions across the path

#### `planning/behavior_tree_planner_orchestrator.hpp`

- `enum class PlanningIntent`
  - `NORMAL`
  - `TIGHT_MANEUVER`

- `struct PlanningRequestContext`
  - `PlanningIntent intent`
  - `std::shared_ptr<zones::Zone> start_zone`
  - `std::shared_ptr<zones::Zone> goal_zone`

- `struct PlanningAttempt`
  - `std::string profile`
  - `std::string transition_profile`
  - `size_t attempt_index`

- `struct PlannerRunResult`
  - `bool success`
  - `std::string detail`

- `using PlannerAttemptRunner = std::function<PlannerRunResult(const PlanningAttempt&)>;`

- `struct BehaviorTreePlanResult`
  - `bool success`
  - `std::string preferred_profile`
  - `std::string selected_profile`
  - `std::string detail`
  - `std::vector<std::string> attempted_profiles`

- `class BehaviorTreePlannerOrchestrator`
  - `BehaviorTreePlannerOrchestrator(std::string xml_filepath, PlannerBehaviorSet behavior_set)`
  - `BehaviorTreePlanResult run(const PlanningRequestContext& request, const PlannerAttemptRunner& runner) const`

- `std::string toString(PlanningIntent intent);`

This is the higher-level profile selection and retry seam. Your app provides a callback that actually runs the planner for each attempted profile.

### `coastmotionplanning::visualizer`

#### `visualizer/planner_visualizer_service.hpp`

DTOs and request/response types:

- `Bounds2d`
- `PointDto`
- `PoseDto`
- `LaneDto`
- `ZoneDto`
- `VehicleFootprintDto`
- `RobotOptionDto`
- `RobotCatalogResponse`
- `MapLoadRequest`
- `MapLoadResponse`
- `PlanRequest`
- `PlanResponse`
- `PlannerVisualizerServiceConfig`

Main service:

- `class PlannerVisualizerService`
  - `explicit PlannerVisualizerService(PlannerVisualizerServiceConfig config)`
  - `RobotCatalogResponse listRobots() const`
  - `MapLoadResponse loadMap(const MapLoadRequest& request)`
  - `PlanResponse plan(const PlanRequest& request)`
  - `static math::Pose2d makePose(const PoseDto& pose_input, const std::string& label)`
  - `static Bounds2d computeBounds(const std::vector<std::shared_ptr<zones::Zone>>& zones)`

Important behavior:

- `loadMap()` expects the map YAML content in `MapLoadRequest::yaml`; `filename` is just a label for display/errors
- only car definitions are currently plan-capable through this service
- `plan()` internally creates a `BehaviorTreePlannerOrchestrator` and `HybridAStarPlanner`
- when debug mode is enabled, `PlanResponse::debug_report_path` points to a generated JSON report on disk

#### `visualizer/planner_visualizer_server.hpp`

- `struct PlannerVisualizerServerConfig`
  - `std::string host`
  - `int port`
  - `std::filesystem::path asset_dir`

- `class PlannerVisualizerServer`
  - `PlannerVisualizerServer(PlannerVisualizerServerConfig config, std::shared_ptr<PlannerVisualizerService> service)`
  - destructor stops the server
  - `int start()`
  - `void stop()`
  - `void wait()`
  - `std::string baseUrl() const`
  - `int port() const`

Behavior notes:

- `port == 0` requests an ephemeral port
- `asset_dir` is optional, but if provided it must exist
- `start()` blocks until `/api/health` succeeds or throws

## 5. Visualizer HTTP API

If you embed `PlannerVisualizerServer`, the currently registered routes are:

### `GET /api/health`

Response:

```json
{"status":"ok"}
```

### `GET /api/robots`

Response body is `RobotCatalogResponse` serialized to JSON:

- `default_robot_name`
- `robots[]`
  - `name`
  - `type`
  - `planning_supported`
  - `vehicle`

### `POST /api/map/load`

Request body:

```json
{
  "filename": "example.yaml",
  "yaml": "maps:\n  name: ...",
  "robot_name": "Pro_XD"
}
```

Response body is `MapLoadResponse` serialized to JSON:

- `map_id`
- `filename`
- `map_name`
- `bounds`
- `vehicle`
- `zones`
- `search_boundary`

### `POST /api/plan`

Request body:

```json
{
  "map_id": "map_1",
  "robot_name": "Pro_XD",
  "start": {"x": 0.0, "y": 0.0, "heading_deg": 0.0},
  "goal": {"x": 10.0, "y": 2.0, "heading_deg": 0.0}
}
```

Response body is `PlanResponse` serialized to JSON:

- `success`
- `detail`
- `selected_profile`
- `attempted_profiles`
- `path`
- `behavior_sequence`
- `segment_directions`
- `debug_mode`
- `debug_report_path`

## 6. Config and file conventions

### Robot file

`configs/robots.yaml` currently models this schema:

- top-level `robots:` sequence
- each robot has `name` and `type`
- `Car` entries use:
  - `geometry.width_m`
  - `geometry.wheelbase_m`
  - `geometry.front_overhang_m`
  - `geometry.rear_overhang_m`
  - `kinematics.max_steer_angle_deg`
  - `kinematics.max_speed_mps`
  - `kinematics.max_reverse_speed_mps`
- `TruckTrailer` entries are validated by `RobotsParser::parse()` but do not currently have a dedicated concrete-definition return type

### Planner behavior files

- `planner_behaviors.yaml` contains:
  - `global.debug_mode`
  - `behaviors.<name>.*`
- `master_params.yaml` is the required reference schema
- every behavior profile must repeat the full schema from `master_params.yaml`
- `PlannerBehaviorSet::loadFromFile()` expects both files to live in the same directory

### Map files

`MapParser` supports:

- top-level `maps` metadata
- top-level `zones` sequence
- zone `type` values currently recognized by the parser:
  - `ManeuveringZone`
  - `TrackMainRoad`
- zone `coordinate_type` values:
  - `world`
  - `lat_long`
  - `long_lat`

`TrackMainRoad` zones also require `lanes`, where each lane provides `lane_waypoints`.

## 7. Helper CLI app

The repository includes a helper executable:

- `generate_nh_lut`

Purpose:

- precompute a dual-model non-holonomic heuristic lookup table for `HybridAStarPlannerRequest::dual_model_lut_path`

CLI parameters:

- `--radius`
- `--angle-bins`
- `--grid-size`
- `--cell-size`
- `--output`

## 8. Current limitations and integration gotchas

- The main planner path is car-only today.
- `RobotsParser` has no `TruckTrailerDefinition` return type.
- `HybridAStarPlanner` takes `robot::Car`, not `robot::RobotBase`.
- `PlannerBehaviorSet::loadFromFile()` silently depends on sibling `master_params.yaml`; if you move `planner_behaviors.yaml`, move the master file with it.
- `CostmapBuilder` always creates its standard layers even if a profile's `active_layers` set is smaller.
- `PlannerVisualizerService` only advertises planning support for car robots.
- The repo does not yet ship `install()`/package-export support, so external apps should consume it through `add_subdirectory` or `FetchContent`.
