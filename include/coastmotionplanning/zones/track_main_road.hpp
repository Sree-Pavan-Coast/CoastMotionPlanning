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

    /// Construct with polygon only.  Validates the polygon geometry.
    TrackMainRoad(const geometry::Polygon2d& polygon,
                  const std::optional<std::string>& name = std::nullopt);

    /// Construct with polygon and lane point sequences.
    /// Validates polygon geometry, requires at least one lane, validates every
    /// lane has >= 2 waypoints, and all waypoints lie inside the polygon.
    TrackMainRoad(const geometry::Polygon2d& polygon,
                  const std::vector<std::vector<geometry::Point2d>>& lane_point_sequences,
                  const std::optional<std::string>& name = std::nullopt);

    const std::vector<std::vector<math::Pose2d>>& getLanes() const { return lanes_; }

    /// Replace all lanes.  Validates that lanes is non-empty, every lane has
    /// >= 2 waypoints, and all waypoints lie inside the polygon.
    void setLanes(const std::vector<std::vector<math::Pose2d>>& lanes);

    /// Computes orientation between consecutive points and stores them as Pose2d.
    /// Validates the lane has >= 2 points and all points lie inside the polygon.
    void addLaneFromPoints(const std::vector<geometry::Point2d>& points);

    std::string getDefaultPlannerBehavior() const override { return ""; }

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

    /// No inherent motion-direction restriction. The selected planner profile
    /// decides whether reverse is allowed for this zone type.
    bool isReverseAllowed() const override { return true; }

private:
    static void validatePolygon(const geometry::Polygon2d& polygon);
    void validateLanePoints(const std::vector<geometry::Point2d>& points) const;
    void validateLaneWaypoints(const std::vector<math::Pose2d>& waypoints) const;

    std::vector<std::vector<math::Pose2d>> lanes_;
};

} // namespace zones
} // namespace coastmotionplanning
