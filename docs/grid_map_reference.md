# GridMap Core Reference & Cheatsheet

`grid_map_core` is a powerful 2.5D mapping library by ANYbotics. This document provides a quick reference for integrating it into the `CoastMotionPlanning` path planning logic.

## 1. Map Initialization
A `GridMap` supports multiple distinct mathematical layers mapped to the same 2D grid space.

```cpp
#include <grid_map_core/grid_map_core.hpp>

// Initialize map with required layers
grid_map::GridMap map({"obstacle_cost", "inflation_cost"});

// Set Map Geometry:
// Length: (x meters, y meters)
// Resolution: meters/cell
// Position: Center of the map in world coordinates (default 0,0)
map.setGeometry(grid_map::Length(10.0, 10.0), 0.1, grid_map::Position(0.0, 0.0));

// Set a common frame id for transforms
map.setFrameId("world");
```

## 2. Coordinate Types
The library strictly enforces types to avoid confusing physical metric coordinates with array indices:
- `grid_map::Position`: Continuous 2D world coordinates (x, y) in meters.
- `grid_map::Index`: Discrete 2D array coordinates (row, column).
- `grid_map::Length`: Length in meters.
- `grid_map::Size`: Size in number of cells.

### Converting Coordinates
```cpp
grid_map::Position position(2.5, 3.2);
grid_map::Index index;

// Find the matrix index for a given world position
if (map.getIndex(position, index)) {
    // position is strictly inside the map bounds
}

// Find the center world position of a specific cell index
grid_map::Position cellCenter;
map.getPosition(index, cellCenter);
```

## 3. Data Accessors
Values in the grid are stored as `float` (not explicit integers), making them ideal for continuous cost values.

```cpp
// Set a default value for an entire layer
map["obstacle_cost"].setConstant(0.0);

// Single cell access by Index (Fastest)
map.at("obstacle_cost", index) = 254.0;

// Single cell access by World Position (Convenient)
map.atPosition("obstacle_cost", position) = 254.0;

// Read a value
float cost = map.atPosition("obstacle_cost", position);
```

## 4. Bulk Matrix Operations (Eigen Integration)
Every layer in `grid_map` is seamlessly backed by an `Eigen::MatrixXf`. This allows for incredibly fast mathematical batch operations on entire layers without looping.

```cpp
// Access the raw Eigen matrix for a layer
auto& obstacleMatrix = map["obstacle_cost"];
auto& inflationMatrix = map["inflation_cost"];

// Bulk operations (e.g., copying or combining layers)
inflationMatrix = obstacleMatrix;

// Mathematical operations across the entire map
inflationMatrix = (obstacleMatrix.array() / 2.0).matrix();

// Extract sub-region matrices
grid_map::Index originX(10, 10);
grid_map::Size patchSize(5, 5);
Eigen::MatrixXf subPatches = obstacleMatrix.block(originX(0), originX(1), patchSize(0), patchSize(1));
```

## 5. Geometric Iterators
Iterators are highly optimized. Use these to apply polygon footprints (like an Ackermann robot) onto the costmap grid.

### Polygon Iterator
Ideal for accurately rasterizing the robot's footprint to check for collisions against obstacles.

```cpp
grid_map::Polygon polygon;
polygon.setFrameId(map.getFrameId());
polygon.addVertex(grid_map::Position(-1.0, -0.5)); // x_min, y_min
polygon.addVertex(grid_map::Position(-1.0, 0.5));  // x_min, y_max
polygon.addVertex(grid_map::Position(1.0, 0.5));   // x_max, y_max
polygon.addVertex(grid_map::Position(1.0, -0.5));  // x_max, y_min

for (grid_map::PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator) {
    if (map.at("obstacle_cost", *iterator) > 100.0) {
        // Collision detected within polygon!
    }
}
```

### Circle/Radius Iterator
Excellent for computing obstacle inflation or distance transform approximations.

```cpp
grid_map::Position center(0.0, 0.0);
double radius = 1.5; // meters

for (grid_map::CircleIterator iterator(map, center, radius); !iterator.isPastEnd(); ++iterator) {
    map.at("inflation_cost", *iterator) = 128.0; 
}
```

### Line Iterator
Excellent for computationally cheap ray-casting between the robot and a goal, or simulating Lidar.

```cpp
grid_map::Index start(0, 0);
grid_map::Index end(50, 50);

for (grid_map::LineIterator iterator(map, start, end); !iterator.isPastEnd(); ++iterator) {
    float cost = map.at("obstacle_cost", *iterator);
}
```

## 6. Shifting and Rolling maps
For a local path planner, keeping the robot strictly centered on the grid without manually shifting millions of memory cells is important. 

`grid_map` tracks physical position using circular buffers under the hood, meaning moving the map window requires **zero memory copying**.

```cpp
grid_map::Position newRobotCenter(10.0, 15.0);

// Shifts the data and coordinate frames while preserving the existing data overlaps.
// Any newly revealed cells will be uninitialized.
map.move(newRobotCenter);
```
