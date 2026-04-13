#pragma once

#include <memory>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace costs {

/// Distance-based cost layer biasing the robot toward lane centerlines
/// in TrackMainRoad zones. ManeuveringZone cells get zero cost.
class LaneCenterlineLayer {
public:
    /// Build the lane_centerline_cost layer.
    /// @param costmap         The grid map to add the layer to
    /// @param selected_zones  The zones selected for this planning query
    /// @param max_lane_cost   Maximum cost at the lane boundary
    /// @param max_half_width  Distance from centerline at which cost saturates
    static void build(grid_map::GridMap& costmap,
                      const std::vector<std::shared_ptr<zones::Zone>>& selected_zones,
                      double max_lane_cost,
                      double max_half_width);
};

} // namespace costs
} // namespace coastmotionplanning
