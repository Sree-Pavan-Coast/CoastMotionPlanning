#pragma once

#include <vector>

#include <grid_map_core/grid_map_core.hpp>

#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/geometry/shape_types.hpp"

namespace coastmotionplanning {
namespace costs {

class DynamicObstacleLayer {
public:
    static void build(grid_map::GridMap& costmap,
                      const std::vector<geometry::Polygon2d>& obstacle_polygons,
                      double inflation_radius_m,
                      double inscribed_radius_m,
                      double cost_scaling_factor);
};

} // namespace costs
} // namespace coastmotionplanning
