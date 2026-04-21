#pragma once

#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/costmap_types.hpp"

namespace coastmotionplanning {
namespace costs {

/// 2D Dijkstra from the goal position on the obstacle grid.
/// Provides an admissible heuristic for A* that accounts for obstacles.
class HolonomicObstaclesHeuristic {
public:
    /// Compute a holonomic heuristic field seeded from one or more world positions.
    /// Cells unreachable from all valid seeds remain NaN.
    static grid_map::Matrix computeField(
        const grid_map::GridMap& costmap,
        const std::vector<grid_map::Position>& seed_positions);

    /// Compute the holonomic-with-obstacles heuristic layer.
    /// @param costmap       The grid map (must contain "static_obstacles")
    /// @param goal_position World position of the goal
    static void compute(grid_map::GridMap& costmap,
                        const grid_map::Position& goal_position);

    /// Compute the holonomic-with-obstacles heuristic layer from a seed set.
    static void compute(grid_map::GridMap& costmap,
                        const std::vector<grid_map::Position>& seed_positions);
};

} // namespace costs
} // namespace coastmotionplanning
