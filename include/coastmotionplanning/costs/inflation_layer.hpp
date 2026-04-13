#pragma once

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/costmap_types.hpp"

namespace coastmotionplanning {
namespace costs {

/// BFS wavefront inflation from lethal cells. Produces a cost gradient
/// that decays exponentially with distance from obstacles.
class InflationLayer {
public:
    /// Build the inflation layer from the static_obstacles layer.
    /// @param costmap              The grid map (must already contain "static_obstacles")
    /// @param inflation_radius_m   Outer radius beyond which cost is 0
    /// @param inscribed_radius_m   Inner radius within which cost is INSCRIBED
    /// @param cost_scaling_factor  Exponential decay factor
    static void build(grid_map::GridMap& costmap,
                      double inflation_radius_m,
                      double inscribed_radius_m,
                      double cost_scaling_factor);
};

} // namespace costs
} // namespace coastmotionplanning
