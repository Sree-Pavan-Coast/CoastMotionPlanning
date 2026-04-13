#pragma once

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/geometry/shape_types.hpp"

namespace coastmotionplanning {
namespace costs {

/// Rasterizes the search boundary polygon onto the grid.
/// Cells outside the boundary are marked LETHAL, cells inside are FREE.
class StaticObstacleLayer {
public:
    /// Build the static_obstacles layer.
    /// @param costmap              The grid map to add the layer to
    /// @param search_boundary      The concave hull polygon defining the operational area
    static void build(grid_map::GridMap& costmap,
                      const geometry::Polygon2d& search_boundary);
};

} // namespace costs
} // namespace coastmotionplanning
