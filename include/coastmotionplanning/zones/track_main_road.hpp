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

    /// Construct with polygon only for staged construction. A valid track road
    /// must later be populated with exactly two opposite-direction lanes.
    TrackMainRoad(const geometry::Polygon2d& polygon,
                  const std::optional<std::string>& name = std::nullopt);

    /// Construct with polygon and lane point sequences.
    /// Validates polygon geometry and requires exactly two opposite-direction
    /// lanes whose waypoints all lie inside the polygon.
    TrackMainRoad(const geometry::Polygon2d& polygon,
                  const std::vector<std::vector<geometry::Point2d>>& lane_point_sequences,
                  const std::optional<std::string>& name = std::nullopt);

    const std::vector<std::vector<math::Pose2d>>& getLanes() const { return lanes_; }

    /// Replace all lanes. Validates that there are exactly two lanes, each has
    /// >= 2 waypoints inside the polygon, and the pair runs in opposite directions.
    void setLanes(const std::vector<std::vector<math::Pose2d>>& lanes);

    /// Computes orientation between consecutive points and stores them as Pose2d.
    /// Validates the lane has >= 2 points inside the polygon. Adding the second
    /// lane validates the opposite-direction pair; adding a third lane throws.
    void addLaneFromPoints(const std::vector<geometry::Point2d>& points);

    std::string getDefaultPlannerBehavior() const override { return ""; }

    /// Track road zones use the lane centerline cost layer for planner biasing.
    /// Internal lane metadata layers are built alongside the centerline layer.
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
    void validateLaneConfiguration(const std::vector<std::vector<math::Pose2d>>& lanes) const;

    std::vector<std::vector<math::Pose2d>> lanes_;
};

} // namespace zones
} // namespace coastmotionplanning
