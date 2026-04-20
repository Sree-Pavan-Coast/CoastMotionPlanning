#pragma once

#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/geometry/shape_types.hpp"

namespace coastmotionplanning {
namespace costs {

/// Rasterizes the search boundary polygon onto the grid.
/// Cells outside the boundary are marked LETHAL, cells inside are FREE.
/// Runtime obstacle polygons are then stamped back to LETHAL inside the boundary.
class StaticObstacleLayer {
public:
    /// Build the static_obstacles layer.
    /// @param costmap              The grid map to add the layer to
    /// @param search_boundary      The exact selected-zone search boundary
    /// @param obstacle_polygons    Runtime obstacle polygons to stamp as lethal cells
    static void build(
        grid_map::GridMap& costmap,
        const geometry::Polygon2d& search_boundary,
        const std::vector<geometry::Polygon2d>& obstacle_polygons = {});
};

} // namespace costs
} // namespace coastmotionplanning
