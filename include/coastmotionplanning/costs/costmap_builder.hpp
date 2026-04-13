#pragma once

#include <memory>
#include <string>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/costs/non_holonomic_heuristic.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/robot/robot_base.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace costs {

/// Orchestrates the building of all costmap layers for a single planning query.
/// Call build() once per planning iteration with the current start/goal.
class CostmapBuilder {
public:
    /// @param config     Costmap configuration
    /// @param all_zones  All parsed zones from the map
    /// @param robot      Robot model (used for inscribed radius)
    CostmapBuilder(const CostmapConfig& config,
                   const std::vector<std::shared_ptr<zones::Zone>>& all_zones,
                   const robot::RobotBase& robot);

    /// Build the full costmap for a planning query.
    /// @param start  Start pose
    /// @param goal   Goal pose
    /// @return Fully populated GridMap with all 7 layers
    grid_map::GridMap build(const math::Pose2d& start, const math::Pose2d& goal);

    /// Load a non-holonomic heuristic LUT from a binary file.
    /// @param filepath Path to the .bin file
    /// @return true if loaded successfully
    bool loadNonHolonomicHeuristic(const std::string& filepath);

    /// Access the loaded non-holonomic heuristic (for planner queries)
    const NonHolonomicHeuristic& getNonHolonomicHeuristic() const { return nh_heuristic_; }

    /// Access the last-built costmap
    const grid_map::GridMap& getCostmap() const { return costmap_; }

private:
    CostmapConfig config_;
    std::vector<std::shared_ptr<zones::Zone>> all_zones_;
    const robot::RobotBase& robot_;
    NonHolonomicHeuristic nh_heuristic_;
    grid_map::GridMap costmap_;
};

} // namespace costs
} // namespace coastmotionplanning
