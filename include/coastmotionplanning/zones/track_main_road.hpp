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

    const std::vector<std::vector<math::Pose2d>>& getLanes() const { return lanes_; }
    void setLanes(const std::vector<std::vector<math::Pose2d>>& lanes) { lanes_ = lanes; }

    // Computes orientation between consecutive points and stores them as Pose2d
    void addLaneFromPoints(const std::vector<geometry::Point2d>& points);

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
    std::vector<std::vector<math::Pose2d>> lanes_;
};

} // namespace zones
} // namespace coastmotionplanning
