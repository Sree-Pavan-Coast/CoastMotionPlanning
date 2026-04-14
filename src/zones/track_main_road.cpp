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

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLaneDirectionToleranceRad = (5.0 * kPi) / 6.0;
constexpr double kDirectionEpsilon = 1e-9;

double normalizeAngleSigned(double angle) {
    angle = std::fmod(angle + kPi, 2.0 * kPi);
    if (angle < 0.0) {
        angle += 2.0 * kPi;
    }
    return angle - kPi;
}

double representativeLaneHeading(const std::vector<math::Pose2d>& waypoints) {
    if (waypoints.size() < 2) {
        throw std::invalid_argument("TrackMainRoad lane must have at least 2 waypoints.");
    }

    const double dx = waypoints.back().x - waypoints.front().x;
    const double dy = waypoints.back().y - waypoints.front().y;
    if (std::hypot(dx, dy) <= kDirectionEpsilon) {
        throw std::invalid_argument(
            "TrackMainRoad lane start and end must define a non-zero travel direction.");
    }

    return std::atan2(dy, dx);
}

std::vector<math::Pose2d> makeLaneWaypoints(const std::vector<geometry::Point2d>& points) {
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

    return lane_waypoints;
}

} // namespace

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

void TrackMainRoad::validateLaneConfiguration(
    const std::vector<std::vector<math::Pose2d>>& lanes) const {
    if (lanes.size() != 2) {
        throw std::invalid_argument(
            "TrackMainRoad requires exactly 2 lanes (got " +
            std::to_string(lanes.size()) + ").");
    }

    for (const auto& lane : lanes) {
        validateLaneWaypoints(lane);
    }

    const double heading0 = representativeLaneHeading(lanes[0]);
    const double heading1 = representativeLaneHeading(lanes[1]);
    const double heading_delta = std::abs(normalizeAngleSigned(heading0 - heading1));
    if (heading_delta < kLaneDirectionToleranceRad) {
        throw std::invalid_argument(
            "TrackMainRoad lanes must run in opposite directions.");
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

    std::vector<std::vector<math::Pose2d>> lanes;
    lanes.reserve(lane_point_sequences.size());
    for (const auto& lane_points : lane_point_sequences) {
        validateLanePoints(lane_points);
        lanes.push_back(makeLaneWaypoints(lane_points));
    }

    setLanes(lanes);
}

// =============================================================================
// Mutators
// =============================================================================

void TrackMainRoad::setLanes(const std::vector<std::vector<math::Pose2d>>& lanes) {
    validateLaneConfiguration(lanes);
    lanes_ = lanes;
}

void TrackMainRoad::addLaneFromPoints(const std::vector<geometry::Point2d>& points) {
    if (lanes_.size() >= 2) {
        throw std::invalid_argument("TrackMainRoad cannot contain more than 2 lanes.");
    }

    validateLanePoints(points);
    lanes_.push_back(makeLaneWaypoints(points));
    if (lanes_.size() == 2) {
        validateLaneConfiguration(lanes_);
    }
}

} // namespace zones
} // namespace coastmotionplanning
