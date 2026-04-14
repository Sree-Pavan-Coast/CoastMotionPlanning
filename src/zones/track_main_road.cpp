#include "coastmotionplanning/zones/track_main_road.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>

namespace coastmotionplanning {
namespace zones {

// =============================================================================
// Polygon validation
// =============================================================================

void TrackMainRoad::validatePolygon(const geometry::Polygon2d& polygon) {
    // Count unique vertices (exclude the Boost.Geometry closing duplicate).
    const auto& outer = polygon.outer();
    size_t vertex_count = outer.size();
    if (vertex_count > 1) {
        const auto& first = outer.front();
        const auto& last = outer.back();
        if (std::abs(geometry::bg::get<0>(first) - geometry::bg::get<0>(last)) < 1e-9 &&
            std::abs(geometry::bg::get<1>(first) - geometry::bg::get<1>(last)) < 1e-9) {
            --vertex_count;
        }
    }

    if (vertex_count < 3) {
        throw std::invalid_argument(
            "TrackMainRoad polygon must have at least 3 vertices (got " +
            std::to_string(vertex_count) + ").");
    }

    // Correct winding order / closing vertex so area and is_valid work.
    geometry::Polygon2d corrected = polygon;
    geometry::bg::correct(corrected);

    double area = geometry::bg::area(corrected);
    if (std::abs(area) < 1e-9) {
        throw std::invalid_argument(
            "TrackMainRoad polygon has zero area (vertices are collinear or duplicated).");
    }

    std::string reason;
    if (!geometry::bg::is_valid(corrected, reason)) {
        throw std::invalid_argument(
            "TrackMainRoad polygon is geometrically invalid: " + reason);
    }
}

// =============================================================================
// Lane-point validation (raw Point2d before conversion to Pose2d)
// =============================================================================

void TrackMainRoad::validateLanePoints(
    const std::vector<geometry::Point2d>& points) const {
    if (points.size() < 2) {
        throw std::invalid_argument(
            "TrackMainRoad lane must have at least 2 waypoints (got " +
            std::to_string(points.size()) + ").");
    }

    geometry::Polygon2d corrected = polygon_;
    geometry::bg::correct(corrected);

    for (size_t i = 0; i < points.size(); ++i) {
        if (!geometry::bg::covered_by(points[i], corrected)) {
            throw std::invalid_argument(
                "TrackMainRoad lane waypoint " + std::to_string(i) +
                " (" + std::to_string(geometry::bg::get<0>(points[i])) +
                ", " + std::to_string(geometry::bg::get<1>(points[i])) +
                ") lies outside the zone polygon.");
        }
    }
}

// =============================================================================
// Lane-waypoint validation (already-converted Pose2d)
// =============================================================================

void TrackMainRoad::validateLaneWaypoints(
    const std::vector<math::Pose2d>& waypoints) const {
    if (waypoints.size() < 2) {
        throw std::invalid_argument(
            "TrackMainRoad lane must have at least 2 waypoints (got " +
            std::to_string(waypoints.size()) + ").");
    }

    geometry::Polygon2d corrected = polygon_;
    geometry::bg::correct(corrected);

    for (size_t i = 0; i < waypoints.size(); ++i) {
        geometry::Point2d pt(waypoints[i].x, waypoints[i].y);
        if (!geometry::bg::covered_by(pt, corrected)) {
            throw std::invalid_argument(
                "TrackMainRoad lane waypoint " + std::to_string(i) +
                " (" + std::to_string(waypoints[i].x) +
                ", " + std::to_string(waypoints[i].y) +
                ") lies outside the zone polygon.");
        }
    }
}

// =============================================================================
// Constructors
// =============================================================================

TrackMainRoad::TrackMainRoad(const geometry::Polygon2d& polygon,
                             const std::optional<std::string>& name)
    : Zone(polygon, name) {
    validatePolygon(polygon);
}

TrackMainRoad::TrackMainRoad(
    const geometry::Polygon2d& polygon,
    const std::vector<std::vector<geometry::Point2d>>& lane_point_sequences,
    const std::optional<std::string>& name)
    : Zone(polygon, name) {
    validatePolygon(polygon);

    if (lane_point_sequences.empty()) {
        throw std::invalid_argument(
            "TrackMainRoad requires at least one lane.");
    }

    for (size_t lane_idx = 0; lane_idx < lane_point_sequences.size(); ++lane_idx) {
        addLaneFromPoints(lane_point_sequences[lane_idx]);
    }
}

// =============================================================================
// Mutators
// =============================================================================

void TrackMainRoad::setLanes(const std::vector<std::vector<math::Pose2d>>& lanes) {
    if (lanes.empty()) {
        throw std::invalid_argument(
            "TrackMainRoad requires at least one lane.");
    }
    for (size_t lane_idx = 0; lane_idx < lanes.size(); ++lane_idx) {
        validateLaneWaypoints(lanes[lane_idx]);
    }
    lanes_ = lanes;
}

void TrackMainRoad::addLaneFromPoints(const std::vector<geometry::Point2d>& points) {
    validateLanePoints(points);

    std::vector<math::Pose2d> lane_waypoints;
    lane_waypoints.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        double yaw = 0.0;
        if (i < points.size() - 1) {
            yaw = std::atan2(points[i + 1].y() - points[i].y(),
                             points[i + 1].x() - points[i].x());
        } else {
            yaw = std::atan2(points[i].y() - points[i - 1].y(),
                             points[i].x() - points[i - 1].x());
        }
        lane_waypoints.emplace_back(points[i].x(), points[i].y(),
                                    math::Angle::from_radians(yaw));
    }

    lanes_.push_back(std::move(lane_waypoints));
}

} // namespace zones
} // namespace coastmotionplanning
