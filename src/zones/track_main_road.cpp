#include "coastmotionplanning/zones/track_main_road.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>

namespace coastmotionplanning {
namespace zones {

namespace {

constexpr double kDirectionEpsilon = 1e-9;
constexpr double kPolygonContainmentTolerance = 1e-6;
constexpr double kMaxJoinOppositionRadians = 120.0 * common::DEG_TO_RAD;

struct NormalizedTrackSegment {
    std::string id;
    double offset{0.0};
    std::vector<geometry::Point2d> points;
    double start_heading{0.0};
    double end_heading{0.0};
};

struct StitchedCenterline {
    std::vector<geometry::Point2d> points;
    std::vector<double> offsets;
};

geometry::Polygon2d correctedPolygon(geometry::Polygon2d polygon) {
    geometry::bg::correct(polygon);
    return polygon;
}

double normalizeAngleSigned(double angle) {
    angle = std::fmod(angle + common::PI, common::TWO_PI);
    if (angle < 0.0) {
        angle += common::TWO_PI;
    }
    return angle - common::PI;
}

double absoluteAngleDifference(double first, double second) {
    return std::abs(normalizeAngleSigned(first - second));
}

double headingBetween(const geometry::Point2d& start, const geometry::Point2d& end) {
    const double dx = end.x() - start.x();
    const double dy = end.y() - start.y();
    if (std::hypot(dx, dy) <= kDirectionEpsilon) {
        throw std::invalid_argument(
            "TrackMainRoad centerline points must define a non-zero travel direction.");
    }
    return std::atan2(dy, dx);
}

double headingFromPoints(const std::vector<geometry::Point2d>& points, size_t index) {
    auto isDistinct = [&](size_t a, size_t b) {
        return !geometry::arePointsClose(points[a], points[b], kDirectionEpsilon);
    };

    size_t previous = index;
    while (previous > 0 && !isDistinct(previous, previous - 1)) {
        --previous;
    }

    size_t next = index;
    while (next + 1 < points.size() && !isDistinct(next, next + 1)) {
        ++next;
    }

    double dx = 0.0;
    double dy = 0.0;
    if (previous > 0 && next + 1 < points.size()) {
        dx = points[next + 1].x() - points[previous - 1].x();
        dy = points[next + 1].y() - points[previous - 1].y();
    } else if (next + 1 < points.size()) {
        dx = points[next + 1].x() - points[index].x();
        dy = points[next + 1].y() - points[index].y();
    } else if (previous > 0) {
        dx = points[index].x() - points[previous - 1].x();
        dy = points[index].y() - points[previous - 1].y();
    }

    if (std::hypot(dx, dy) <= kDirectionEpsilon) {
        throw std::invalid_argument(
            "TrackMainRoad center_waypoints must define a non-zero travel direction.");
    }

    return std::atan2(dy, dx);
}

std::vector<math::Pose2d> makeWaypoints(const std::vector<geometry::Point2d>& points) {
    if (points.size() < 2) {
        throw std::invalid_argument(
            "TrackMainRoad center_waypoints must contain at least 2 points.");
    }

    std::vector<math::Pose2d> waypoints;
    waypoints.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        waypoints.emplace_back(
            points[i].x(),
            points[i].y(),
            math::Angle::from_radians(headingFromPoints(points, i)));
    }
    return waypoints;
}

geometry::Point2d offsetPoint(const math::Pose2d& pose, double offset, double side_sign) {
    const double heading = pose.theta.radians();
    const double right_normal_x = std::sin(heading);
    const double right_normal_y = -std::cos(heading);
    return geometry::Point2d(
        pose.x + (side_sign * offset * right_normal_x),
        pose.y + (side_sign * offset * right_normal_y));
}

bool isCoveredByPolygonWithTolerance(const geometry::Point2d& point,
                                     const geometry::Polygon2d& polygon) {
    return geometry::bg::covered_by(point, polygon) ||
           geometry::bg::distance(point, polygon) <= kPolygonContainmentTolerance;
}

double crossProduct(const geometry::Point2d& a,
                    const geometry::Point2d& b,
                    const geometry::Point2d& c) {
    return (b.x() - a.x()) * (c.y() - a.y()) -
           (b.y() - a.y()) * (c.x() - a.x());
}

bool rangesOverlap(double min_first, double max_first, double min_second, double max_second) {
    return std::max(min_first, min_second) <=
           std::min(max_first, max_second) + kPolygonContainmentTolerance;
}

bool boundingBoxesOverlap(const geometry::Point2d& first_start,
                          const geometry::Point2d& first_end,
                          const geometry::Point2d& second_start,
                          const geometry::Point2d& second_end) {
    return rangesOverlap(
               std::min(first_start.x(), first_end.x()),
               std::max(first_start.x(), first_end.x()),
               std::min(second_start.x(), second_end.x()),
               std::max(second_start.x(), second_end.x())) &&
           rangesOverlap(
               std::min(first_start.y(), first_end.y()),
               std::max(first_start.y(), first_end.y()),
               std::min(second_start.y(), second_end.y()),
               std::max(second_start.y(), second_end.y()));
}

int orientationSign(double value) {
    if (value > kPolygonContainmentTolerance) {
        return 1;
    }
    if (value < -kPolygonContainmentTolerance) {
        return -1;
    }
    return 0;
}

bool isPointOnSegment(const geometry::Point2d& point,
                      const geometry::Point2d& segment_start,
                      const geometry::Point2d& segment_end) {
    if (std::abs(crossProduct(segment_start, segment_end, point)) >
        kPolygonContainmentTolerance) {
        return false;
    }

    return point.x() >= std::min(segment_start.x(), segment_end.x()) -
                            kPolygonContainmentTolerance &&
           point.x() <= std::max(segment_start.x(), segment_end.x()) +
                            kPolygonContainmentTolerance &&
           point.y() >= std::min(segment_start.y(), segment_end.y()) -
                            kPolygonContainmentTolerance &&
           point.y() <= std::max(segment_start.y(), segment_end.y()) +
                            kPolygonContainmentTolerance;
}

bool segmentsIntersect(const geometry::Point2d& first_start,
                       const geometry::Point2d& first_end,
                       const geometry::Point2d& second_start,
                       const geometry::Point2d& second_end) {
    if (!boundingBoxesOverlap(first_start, first_end, second_start, second_end)) {
        return false;
    }

    const double first_second_start = crossProduct(first_start, first_end, second_start);
    const double first_second_end = crossProduct(first_start, first_end, second_end);
    const double second_first_start = crossProduct(second_start, second_end, first_start);
    const double second_first_end = crossProduct(second_start, second_end, first_end);

    const int first_second_start_sign = orientationSign(first_second_start);
    const int first_second_end_sign = orientationSign(first_second_end);
    const int second_first_start_sign = orientationSign(second_first_start);
    const int second_first_end_sign = orientationSign(second_first_end);

    if (first_second_start_sign == 0 && isPointOnSegment(second_start, first_start, first_end)) {
        return true;
    }
    if (first_second_end_sign == 0 && isPointOnSegment(second_end, first_start, first_end)) {
        return true;
    }
    if (second_first_start_sign == 0 && isPointOnSegment(first_start, second_start, second_end)) {
        return true;
    }
    if (second_first_end_sign == 0 && isPointOnSegment(first_end, second_start, second_end)) {
        return true;
    }

    return first_second_start_sign != first_second_end_sign &&
           second_first_start_sign != second_first_end_sign;
}

bool isSegmentCoveredByPolygon(const geometry::Point2d& start,
                               const geometry::Point2d& end,
                               const geometry::Polygon2d& polygon) {
    if (geometry::arePointsClose(start, end, kDirectionEpsilon)) {
        return isCoveredByPolygonWithTolerance(start, polygon);
    }

    geometry::LineString2d segment;
    segment.push_back(start);
    segment.push_back(end);
    if (geometry::bg::covered_by(segment, polygon)) {
        return true;
    }

    constexpr size_t kFallbackSamples = 8;
    const double dx = end.x() - start.x();
    const double dy = end.y() - start.y();
    for (size_t sample_index = 0; sample_index <= kFallbackSamples; ++sample_index) {
        const double t = static_cast<double>(sample_index) /
                         static_cast<double>(kFallbackSamples);
        const geometry::Point2d sample(
            start.x() + (dx * t),
            start.y() + (dy * t));
        if (!isCoveredByPolygonWithTolerance(sample, polygon)) {
            return false;
        }
    }

    return true;
}

void validateSimplePolyline(const std::vector<geometry::Point2d>& points,
                            const std::string& label) {
    for (size_t first_index = 0; first_index + 1 < points.size(); ++first_index) {
        for (size_t second_index = first_index + 2;
             second_index + 1 < points.size();
             ++second_index) {
            if (second_index == first_index + 1) {
                continue;
            }
            if (segmentsIntersect(
                    points[first_index],
                    points[first_index + 1],
                    points[second_index],
                    points[second_index + 1])) {
                throw std::invalid_argument(
                    "TrackMainRoad " + label + " must be a simple polyline.");
            }
        }
    }
}

void validatePolylineWithinPolygon(const std::vector<geometry::Point2d>& points,
                                   const geometry::Polygon2d& polygon,
                                   const std::string& label) {
    for (size_t i = 0; i < points.size(); ++i) {
        if (!isCoveredByPolygonWithTolerance(points[i], polygon)) {
            throw std::invalid_argument(
                "TrackMainRoad " + label + " waypoint " + std::to_string(i) +
                " lies outside the zone polygon.");
        }
    }

    for (size_t i = 0; i + 1 < points.size(); ++i) {
        if (!isSegmentCoveredByPolygon(points[i], points[i + 1], polygon)) {
            throw std::invalid_argument(
                "TrackMainRoad " + label + " segment " + std::to_string(i) +
                " leaves the zone polygon.");
        }
    }
}

std::vector<geometry::Point2d> collapseAdjacentDuplicatePoints(
    const std::vector<geometry::Point2d>& points) {
    std::vector<geometry::Point2d> collapsed;
    collapsed.reserve(points.size());
    for (const auto& point : points) {
        if (collapsed.empty() || !geometry::arePointsClose(collapsed.back(), point)) {
            collapsed.push_back(point);
        }
    }
    return collapsed;
}

NormalizedTrackSegment normalizeSegment(const TrackMainRoadSegment& segment) {
    if (segment.id.empty()) {
        throw std::invalid_argument("TrackMainRoad lane.segments ids must be non-empty.");
    }
    if (!std::isfinite(segment.offset) || segment.offset < 0.0) {
        throw std::invalid_argument(
            "TrackMainRoad lane.segments['" + segment.id +
            "'].offset must be finite and non-negative.");
    }

    auto points = collapseAdjacentDuplicatePoints(segment.center_waypoints);
    if (points.size() < 2) {
        throw std::invalid_argument(
            "TrackMainRoad lane.segments['" + segment.id +
            "'].center_waypoints must contain at least 2 distinct points.");
    }

    const double start_heading = headingBetween(points[0], points[1]);
    const double end_heading = headingBetween(points[points.size() - 2], points.back());

    return NormalizedTrackSegment{
        segment.id,
        segment.offset,
        std::move(points),
        start_heading,
        end_heading
    };
}

double computeJoinHeading(const NormalizedTrackSegment& previous,
                          const NormalizedTrackSegment& next,
                          bool touching) {
    if (touching) {
        return headingBetween(previous.points[previous.points.size() - 2], next.points[1]);
    }
    return headingBetween(previous.points.back(), next.points.front());
}

void validateJoinDirection(const NormalizedTrackSegment& previous,
                           const NormalizedTrackSegment& next,
                           bool touching) {
    const double join_heading = computeJoinHeading(previous, next, touching);
    if (absoluteAngleDifference(join_heading, previous.end_heading) > kMaxJoinOppositionRadians ||
        absoluteAngleDifference(join_heading, next.start_heading) > kMaxJoinOppositionRadians) {
        throw std::invalid_argument(
            "TrackMainRoad segment '" + next.id +
            "' is ordered opposite to neighboring segment '" + previous.id + "'.");
    }
}

StitchedCenterline stitchSegments(const std::vector<NormalizedTrackSegment>& segments) {
    StitchedCenterline stitched;
    stitched.points = segments.front().points;
    stitched.offsets.assign(segments.front().points.size(), segments.front().offset);

    for (size_t i = 1; i < segments.size(); ++i) {
        const auto& previous = segments[i - 1];
        const auto& next = segments[i];
        const bool touching =
            geometry::arePointsClose(stitched.points.back(), next.points.front());
        validateJoinDirection(previous, next, touching);

        const size_t start_index = touching ? 1 : 0;
        for (size_t point_index = start_index; point_index < next.points.size(); ++point_index) {
            if (!stitched.points.empty() &&
                geometry::arePointsClose(stitched.points.back(), next.points[point_index])) {
                continue;
            }
            stitched.points.push_back(next.points[point_index]);
            stitched.offsets.push_back(next.offset);
        }
    }

    return stitched;
}

} // namespace

void TrackMainRoad::validatePolygon(const geometry::Polygon2d& polygon) {
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

    geometry::Polygon2d corrected = polygon;
    geometry::bg::correct(corrected);

    const double area = geometry::bg::area(corrected);
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

void TrackMainRoad::validateCenterlinePoints(const std::vector<geometry::Point2d>& points,
                                             const std::vector<double>& offsets) const {
    validateCenterlineWaypoints(makeWaypoints(points), offsets);
}

void TrackMainRoad::validateCenterlineWaypoints(const std::vector<math::Pose2d>& waypoints,
                                                const std::vector<double>& offsets) const {
    if (waypoints.size() < 2) {
        throw std::invalid_argument(
            "TrackMainRoad lane.center_waypoints must contain at least 2 waypoints (got " +
            std::to_string(waypoints.size()) + ").");
    }
    if (waypoints.size() != offsets.size()) {
        throw std::invalid_argument(
            "TrackMainRoad lane.center_waypoints and lane.offsets must have the same length.");
    }

    const auto corrected = correctedPolygon(polygon_);

    std::vector<geometry::Point2d> center_points;
    center_points.reserve(waypoints.size());
    for (size_t i = 0; i < waypoints.size(); ++i) {
        if (!std::isfinite(offsets[i]) || offsets[i] < 0.0) {
            throw std::invalid_argument(
                "TrackMainRoad lane.offsets[" + std::to_string(i) +
                "] must be finite and non-negative.");
        }

        center_points.emplace_back(waypoints[i].x, waypoints[i].y);
    }

    validateSimplePolyline(center_points, "centerline");
    validatePolylineWithinPolygon(center_points, corrected, "centerline");

    const auto right_points = buildOffsetPolyline(waypoints, offsets, 1.0);
    const auto left_points = buildOffsetPolyline(waypoints, offsets, -1.0);

    validateSimplePolyline(right_points, "derived right lane");
    validateSimplePolyline(left_points, "derived left lane");
    validatePolylineWithinPolygon(right_points, corrected, "derived right lane");
    validatePolylineWithinPolygon(left_points, corrected, "derived left lane");
}

void TrackMainRoad::validateCenterlineSegments(
    const std::vector<TrackMainRoadSegment>& segments) const {
    if (segments.empty()) {
        throw std::invalid_argument(
            "TrackMainRoad lane.segments must contain at least one segment.");
    }

    const auto corrected = correctedPolygon(polygon_);
    std::unordered_set<std::string> segment_ids;
    for (const auto& segment : segments) {
        const auto normalized = normalizeSegment(segment);
        if (!segment_ids.insert(normalized.id).second) {
            throw std::invalid_argument(
                "TrackMainRoad segment id '" + normalized.id + "' must be unique.");
        }
        validatePolylineWithinPolygon(
            normalized.points,
            corrected,
            "segment '" + normalized.id + "' centerline");
    }
}

std::vector<geometry::Point2d> TrackMainRoad::buildOffsetPolyline(double side_sign) const {
    return buildOffsetPolyline(centerline_, offsets_, side_sign);
}

std::vector<geometry::Point2d> TrackMainRoad::buildOffsetPolyline(
    const std::vector<math::Pose2d>& centerline,
    const std::vector<double>& offsets,
    double side_sign) const {
    std::vector<geometry::Point2d> points;
    points.reserve(centerline.size());
    for (size_t i = 0; i < centerline.size(); ++i) {
        points.push_back(offsetPoint(centerline[i], offsets[i], side_sign));
    }
    return points;
}

void TrackMainRoad::rebuildDerivedLanes() {
    const auto right_points = buildOffsetPolyline(1.0);
    auto left_points = buildOffsetPolyline(-1.0);
    std::reverse(left_points.begin(), left_points.end());

    std::vector<std::vector<math::Pose2d>> lanes;
    lanes.reserve(2);
    lanes.push_back(makeWaypoints(right_points));
    lanes.push_back(makeWaypoints(left_points));
    lanes_ = std::move(lanes);
}

TrackMainRoad::TrackMainRoad(const geometry::Polygon2d& polygon,
                             const std::optional<std::string>& name)
    : Zone(polygon, name) {
    validatePolygon(polygon);
}

TrackMainRoad::TrackMainRoad(const geometry::Polygon2d& polygon,
                             const std::vector<TrackMainRoadSegment>& segments,
                             const std::optional<std::string>& name)
    : Zone(polygon, name) {
    validatePolygon(polygon);
    setCenterlineSegments(segments);
}

TrackMainRoad::TrackMainRoad(const geometry::Polygon2d& polygon,
                             const std::vector<geometry::Point2d>& centerline_points,
                             const std::vector<double>& offsets,
                             const std::optional<std::string>& name)
    : Zone(polygon, name) {
    validatePolygon(polygon);
    setCenterlineFromPoints(centerline_points, offsets);
}

void TrackMainRoad::setCenterline(const std::vector<math::Pose2d>& centerline,
                                  const std::vector<double>& offsets) {
    validateCenterlineWaypoints(centerline, offsets);
    centerline_ = centerline;
    offsets_ = offsets;
    rebuildDerivedLanes();
}

void TrackMainRoad::setCenterlineSegments(
    const std::vector<TrackMainRoadSegment>& segments) {
    validateCenterlineSegments(segments);

    std::vector<NormalizedTrackSegment> normalized_segments;
    normalized_segments.reserve(segments.size());
    for (const auto& segment : segments) {
        normalized_segments.push_back(normalizeSegment(segment));
    }

    const auto stitched = stitchSegments(normalized_segments);
    setCenterline(makeWaypoints(stitched.points), stitched.offsets);
}

void TrackMainRoad::setCenterlineFromPoints(const std::vector<geometry::Point2d>& points,
                                            const std::vector<double>& offsets) {
    validateCenterlinePoints(points, offsets);
    setCenterline(makeWaypoints(points), offsets);
}

} // namespace zones
} // namespace coastmotionplanning
