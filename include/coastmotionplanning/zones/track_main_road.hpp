#pragma once

#include <vector>
#include <string>
#include "coastmotionplanning/zones/zone.hpp"
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/math/pose2d.hpp"

namespace coastmotionplanning {
namespace zones {

class TrackMainRoad : public Zone {
public:
    TrackMainRoad() = default;
    ~TrackMainRoad() override = default;

    TrackMainRoad(const geometry::Polygon2d& polygon, const std::optional<std::string>& name = std::nullopt);

    const std::vector<math::Pose2d>& getLaneWaypoints() const { return lane_waypoints_; }
    void setLaneWaypoints(const std::vector<math::Pose2d>& waypoints) { lane_waypoints_ = waypoints; }
    void addLaneWaypoint(const math::Pose2d& waypoint) { lane_waypoints_.push_back(waypoint); }

    // Computes orientation between consecutive points and stores them as Pose2d
    void setLaneWaypointsFromPoints(const std::vector<geometry::Point2d>& points);

    std::string getDefaultPlannerBehavior() const override { return "primary_profile"; }

    /// Track road zones activate: static_obstacles, inflation, lane_centerline_cost,
    /// holonomic heuristic. Lane centerline biases the robot toward lane centers.
    std::vector<std::string> getActiveLayers() const override {
        return {
            costs::CostmapLayerNames::STATIC_OBSTACLES,
            costs::CostmapLayerNames::INFLATION,
            costs::CostmapLayerNames::LANE_CENTERLINE_COST,
            costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES
        };
    }

    /// Only forward motion is allowed in a track road zone.
    bool isReverseAllowed() const override { return false; }

private:
    std::vector<math::Pose2d> lane_waypoints_;
};

} // namespace zones
} // namespace coastmotionplanning
