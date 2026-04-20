#pragma once

#include <optional>
#include <string>
#include <vector>

#include "coastmotionplanning/zones/zone.hpp"
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/math/pose2d.hpp"

namespace coastmotionplanning {
namespace zones {

struct TrackMainRoadSegment {
    std::string id;
    double offset{0.0};
    std::vector<geometry::Point2d> center_waypoints;
};

class TrackMainRoad : public Zone {
public:
    TrackMainRoad() = default;
    ~TrackMainRoad() override = default;

    /// Construct with polygon only for staged construction. A valid track road
    /// must later be populated with centerline segments or a canonical
    /// centerline and per-waypoint offsets.
    TrackMainRoad(const geometry::Polygon2d& polygon,
                  const std::optional<std::string>& name = std::nullopt);

    /// Construct with polygon and ordered authoring segments. Segment list order
    /// defines travel order; gaps are stitched into one canonical centerline.
    TrackMainRoad(const geometry::Polygon2d& polygon,
                  const std::vector<TrackMainRoadSegment>& segments,
                  const std::optional<std::string>& name = std::nullopt);

    /// Construct with polygon, ordered canonical centerline points, and
    /// per-waypoint offsets. Retained for tests and internal callers.
    TrackMainRoad(const geometry::Polygon2d& polygon,
                  const std::vector<geometry::Point2d>& centerline_points,
                  const std::vector<double>& offsets,
                  const std::optional<std::string>& name = std::nullopt);

    const std::vector<math::Pose2d>& getCenterline() const { return centerline_; }
    const std::vector<double>& getOffsets() const { return offsets_; }
    const std::vector<std::vector<math::Pose2d>>& getLanes() const { return lanes_; }

    /// Replace the canonical track-road centerline. Validates centerline and
    /// derived lane geometry before publishing the new representation.
    void setCenterline(const std::vector<math::Pose2d>& centerline,
                       const std::vector<double>& offsets);

    /// Replace the track-road authoring segments. Segments are stitched into a
    /// canonical centerline before validation and publication.
    void setCenterlineSegments(const std::vector<TrackMainRoadSegment>& segments);

    /// Computes heading along the centerline and stores it as Pose2d.
    void setCenterlineFromPoints(const std::vector<geometry::Point2d>& points,
                                 const std::vector<double>& offsets);

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
    void validateCenterlinePoints(const std::vector<geometry::Point2d>& points,
                                  const std::vector<double>& offsets) const;
    void validateCenterlineWaypoints(const std::vector<math::Pose2d>& waypoints,
                                     const std::vector<double>& offsets) const;
    void validateCenterlineSegments(const std::vector<TrackMainRoadSegment>& segments) const;
    std::vector<geometry::Point2d> buildOffsetPolyline(double side_sign) const;
    std::vector<geometry::Point2d> buildOffsetPolyline(
        const std::vector<math::Pose2d>& centerline,
        const std::vector<double>& offsets,
        double side_sign) const;
    void rebuildDerivedLanes();

    std::vector<math::Pose2d> centerline_;
    std::vector<double> offsets_;
    std::vector<std::vector<math::Pose2d>> lanes_;
};

} // namespace zones
} // namespace coastmotionplanning
