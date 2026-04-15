#pragma once

#include <limits>
#include <string>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/costmap_types.hpp"

namespace coastmotionplanning {
namespace costs {

struct HolonomicHeuristicLayerStats {
    size_t seed_cell_count{0};
    size_t finite_cell_count{0};
    double min_finite_value{std::numeric_limits<double>::quiet_NaN()};
    double max_finite_value{std::numeric_limits<double>::quiet_NaN()};
};

/// 2D Dijkstra from the goal position on the obstacle grid.
/// Provides an admissible heuristic for A* that accounts for obstacles.
class HolonomicObstaclesHeuristic {
public:
    /// Compute the holonomic-with-obstacles heuristic layer.
    /// @param costmap       The grid map (must contain "static_obstacles")
    /// @param goal_position World position of the goal
    static HolonomicHeuristicLayerStats compute(
        grid_map::GridMap& costmap,
        const grid_map::Position& goal_position);

    /// Compute a heuristic layer from an explicit set of seed cells.
    static HolonomicHeuristicLayerStats computeFromSeedCells(
        grid_map::GridMap& costmap,
        const std::string& layer_name,
        const std::vector<grid_map::Index>& seed_indices);

    /// Stable layer naming for frontier-stage heuristic layers.
    static std::string makeStageLayerName(size_t source_frontier_id,
                                          size_t target_frontier_id);
};

} // namespace costs
} // namespace coastmotionplanning
