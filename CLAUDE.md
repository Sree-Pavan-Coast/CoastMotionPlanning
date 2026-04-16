# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Test Commands

```bash
# Configure and build (from repo root)
cmake -S . -B build
cmake --build build -j$(sysctl -n hw.ncpu)

# Run all tests
cd build && ctest --output-on-failure

# Run a single test executable (test targets listed in tests/CMakeLists.txt)
./build/collision_checker_test
./build/inflation_layer_test
./build/costmap_builder_test
# etc.

# Run a single test case by name
./build/costmap_builder_test --gtest_filter="CostmapBuilderTest.BuildProducesAllLayers"

# Generate non-holonomic heuristic lookup table
./build/generate_nh_lut --radius 8.0 --angle-bins 72 --grid-size 400 --cell-size 0.1 --output nh_lut.bin
```

## Architecture

This is a zone-aware Hybrid A* motion planner for Ackermann-steered vehicles (cars and truck-trailers). C++17, CMake, no ROS dependency. All external deps (Eigen3, yaml-cpp, grid_map_core, GTest) are fetched via CMake FetchContent.

### Planning Pipeline

**Map Parser** (`map/`) reads YAML map files containing zone polygons with optional geographic coordinates (lat/long converted via `CoordinateTransform`). Each zone becomes a polymorphic `Zone` subclass.

**Zone System** (`zones/`) controls planner behavior per-region. `Zone` is the abstract base; two concrete types:
- `ManeuveringZone` -- allows forward+reverse motion, no lane centerline cost
- `TrackMainRoad` -- forward-only, adds lane centerline cost layer, has lane waypoints

Each zone declares which costmap layers it activates (`getActiveLayers()`) and whether reverse is allowed (`isReverseAllowed()`). The planner queries these at runtime.

**Costmap Builder** (`costs/`) orchestrates multi-layer costmap construction on `grid_map::GridMap`:
- `static_obstacles` -- binary free/lethal from search boundary
- `inflation` -- BFS wavefront decay from lethal cells
- `zone_constraints` -- zone index per cell (255 = outside all zones = lethal)
- `lane_centerline_cost` -- distance-to-centerline in track road zones
- `holonomic_with_obstacles` -- 2D Dijkstra heuristic from goal
- `combined_cost` -- merged layer for planner consumption

`ZoneSelector` picks active zones for a start/goal pair. Layer building is filtered by both the planner behavior profile's `active_layers` and each zone's `getActiveLayers()`.

**Robot Models** (`robot/`) derive from `RobotBase`. `Car` is a single rigid body with Ackermann steering. `TruckTrailer` is an articulated tractor+trailer with hitch joint. Both provide `getRobotFootprint()` for collision checking. Robot geometry is defined in `configs/robots.yaml`.

**Motion Primitives** (`motion_primitives/`) precompute discrete motion endpoints per heading bin. `CarMotionTable` generates primitives for single-body vehicles. `TruckTrailerMotionTable` extends this for articulated vehicles. Coordinates are in cell units, not meters.

**Non-Holonomic Heuristic** (`costs/non_holonomic_heuristic`) uses a precomputed 3D Dijkstra LUT (x, y, theta) over Reeds-Shepp arcs. Generated offline by the `generate_nh_lut` app. Exploits y-axis symmetry (stores only y >= 0).

**Collision Checker** (`collision_checking/`) tests robot footprint polygons against the costmap using Boost.Geometry.

### Configuration

- `configs/planner_behaviors.yaml` -- embedded schema plus named profiles (primary, relaxed, parking) and active costmap layers
- `configs/robots.yaml` -- robot geometry and kinematic definitions
- `configs/maps/*.yaml` -- map definitions with zone polygons

### Key Types

- Geometry primitives (`geometry/shape_types.hpp`) are Boost.Geometry types: `Point2d`, `Polygon2d`, `Box2d`, `LineString2d`
- Cost values (`costs/costmap_types.hpp`) follow ROS nav2_costmap_2d conventions (0=free, 254=lethal, 255=no info)
- Motion primitives use cell-unit coordinates, not meters

## Code Style

Enforced by `.clang-format` (Google-based, 4-space indent, 100-col limit, braces after functions) and `.clang-tidy`. Naming conventions:
- Classes/Structs: `CamelCase`
- Functions/methods: `camelBack`
- Variables/parameters/members: `camelBack`
- Namespaces: `lower_case`
- Member variables use trailing underscore: `member_name_`

All code lives under the `coastmotionplanning` namespace with sub-namespaces matching directory structure (`costs`, `zones`, `robot`, `math`, `geometry`, `map`, `motion_primitives`, `collision_checking`, `common`).
