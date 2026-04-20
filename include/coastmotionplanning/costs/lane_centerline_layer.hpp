#pragma once

#include <memory>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/costs/zone_selector.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace costs {

enum class TrackLaneGuidanceTargetKind {
    Goal,
    Handoff
};

struct TrackLaneGuidance {
    size_t frontier_id{0};
    double target_station{0.0};
    TrackLaneGuidanceTargetKind target_kind{TrackLaneGuidanceTargetKind::Goal};
};

/// Distance-based cost layer biasing the robot toward lane centerlines
/// in TrackMainRoad zones. ManeuveringZone cells get zero cost.
class LaneCenterlineLayer {
public:
    /// Build the lane_centerline_cost layer.
    /// @param costmap         The grid map to add the layer to
    /// @param selection       The selected frontier descriptors and search boundary
    /// @param start           The planning start pose
    /// @param goal            The planning goal pose
    /// @param max_lane_cost   Maximum cost at the lane boundary
    /// @param max_half_width  Distance from centerline at which cost saturates
    static std::vector<TrackLaneGuidance> build(
        grid_map::GridMap& costmap,
        const ZoneSelectionResult& selection,
        const math::Pose2d& start,
        const math::Pose2d& goal,
        double max_lane_cost,
        double max_half_width);
};

} // namespace costs
} // namespace coastmotionplanning
