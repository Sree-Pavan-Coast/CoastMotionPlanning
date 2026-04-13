#pragma once

#include <memory>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/geometry/shape_types.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace costs {

/// Categorical layer encoding which zone type each cell belongs to.
/// Used by the planner to decide if reverse motion is allowed.
class ZoneConstraintsLayer {
public:
    /// Build the zone_constraints layer.
    /// @param costmap         The grid map to add the layer to
    /// @param selected_zones  The zones selected for this planning query
    /// @param search_boundary The concave hull search boundary
    static void build(grid_map::GridMap& costmap,
                      const std::vector<std::shared_ptr<zones::Zone>>& selected_zones,
                      const geometry::Polygon2d& search_boundary);
};

} // namespace costs
} // namespace coastmotionplanning
