#include "coastmotionplanning/costs/lane_centerline_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

#include "coastmotionplanning/zones/track_main_road.hpp"

namespace coastmotionplanning {
namespace costs {

namespace {

using MultiLineString2d = geometry::bg::model::multi_linestring<geometry::LineString2d>;
using MultiPolygon2d = geometry::bg::model::multi_polygon<geometry::Polygon2d>;

constexpr double kProjectionEpsilon = 1e-12;
constexpr double kOverlapAreaTolerance = 1e-9;
constexpr double kCandidateDedupTolerance = 1e-6;

struct PolylineSegment {
    geometry::Point2d start;
    geometry::Point2d end;
    double heading_rad{0.0};
    double length_sq{0.0};
    double length_m{0.0};
    double station_start_m{0.0};
};

struct PolylineProjection {
    bool valid{false};
    geometry::Point2d nearest_point;
    double distance_sq{std::numeric_limits<double>::infinity()};
    double station_m{0.0};
    double heading_rad{0.0};
};

struct TrackInfo {
    size_t frontier_id{0};
    std::shared_ptr<zones::TrackMainRoad> track;
    std::vector<PolylineSegment> centerline_segments;
    std::array<std::vector<PolylineSegment>, 2> lane_segments;
    grid_map::Polygon gm_polygon;
    double target_station_m{0.0};
    TrackLaneGuidanceTargetKind target_kind{TrackLaneGuidanceTargetKind::Goal};
};

double squaredDistance(const geometry::Point2d& first, const geometry::Point2d& second) {
    const double dx = first.x() - second.x();
    const double dy = first.y() - second.y();
    return (dx * dx) + (dy * dy);
}

std::vector<PolylineSegment> buildPolylineSegments(
    const std::vector<math::Pose2d>& polyline) {
    std::vector<PolylineSegment> segments;
    double station = 0.0;
    for (size_t idx = 0; idx + 1 < polyline.size(); ++idx) {
        const geometry::Point2d start(polyline[idx].x, polyline[idx].y);
        const geometry::Point2d end(polyline[idx + 1].x, polyline[idx + 1].y);
        const double dx = end.x() - start.x();
        const double dy = end.y() - start.y();
        const double length_sq = (dx * dx) + (dy * dy);
        if (length_sq <= kProjectionEpsilon) {
            continue;
        }

        const double length_m = std::sqrt(length_sq);
        PolylineSegment segment;
        segment.start = start;
        segment.end = end;
        segment.heading_rad = std::atan2(dy, dx);
        segment.length_sq = length_sq;
        segment.length_m = length_m;
        segment.station_start_m = station;
        segments.push_back(segment);
        station += length_m;
    }
    return segments;
}

PolylineProjection projectOntoSegments(
    const geometry::Point2d& point,
    const std::vector<PolylineSegment>& segments) {
    PolylineProjection best;
    for (const auto& segment : segments) {
        if (segment.length_sq <= kProjectionEpsilon) {
            continue;
        }

        const double px = point.x();
        const double py = point.y();
        const double x0 = segment.start.x();
        const double y0 = segment.start.y();
        const double x1 = segment.end.x();
        const double y1 = segment.end.y();
        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double t = std::clamp(
            (((px - x0) * dx) + ((py - y0) * dy)) / segment.length_sq,
            0.0,
            1.0);
        const geometry::Point2d nearest_point(x0 + (t * dx), y0 + (t * dy));
        const double distance_sq = squaredDistance(point, nearest_point);
        if (best.valid && distance_sq >= best.distance_sq) {
            continue;
        }

        best.valid = true;
        best.nearest_point = nearest_point;
        best.distance_sq = distance_sq;
        best.station_m = segment.station_start_m + (t * segment.length_m);
        best.heading_rad = segment.heading_rad;
    }

    return best;
}

grid_map::Polygon buildGridMapPolygon(const geometry::Polygon2d& polygon) {
    grid_map::Polygon gm_polygon;
    const auto& outer = polygon.outer();
    for (size_t idx = 0; idx < outer.size(); ++idx) {
        if (idx + 1 == outer.size() &&
            std::abs(geometry::bg::get<0>(outer.front()) - geometry::bg::get<0>(outer.back())) <
                1e-9 &&
            std::abs(geometry::bg::get<1>(outer.front()) - geometry::bg::get<1>(outer.back())) <
                1e-9) {
            continue;
        }
        gm_polygon.addVertex(
            grid_map::Position(geometry::bg::get<0>(outer[idx]), geometry::bg::get<1>(outer[idx])));
    }
    return gm_polygon;
}

geometry::Polygon2d correctedPolygon(geometry::Polygon2d polygon) {
    geometry::bg::correct(polygon);
    return polygon;
}

geometry::LineString2d boundaryLineString(const geometry::Polygon2d& polygon) {
    geometry::LineString2d boundary;
    const auto& outer = polygon.outer();
    if (outer.empty()) {
        return boundary;
    }

    for (const auto& point : outer) {
        boundary.push_back(point);
    }
    if (!geometry::arePointsClose(boundary.front(), boundary.back())) {
        boundary.push_back(boundary.front());
    }
    return boundary;
}

void appendUniqueCandidate(std::vector<geometry::Point2d>& candidates,
                           const geometry::Point2d& candidate) {
    const auto duplicate = std::any_of(
        candidates.begin(),
        candidates.end(),
        [&](const auto& existing) {
            return geometry::arePointsClose(existing, candidate, kCandidateDedupTolerance);
        });
    if (!duplicate) {
        candidates.push_back(candidate);
    }
}

void appendLineCandidates(std::vector<geometry::Point2d>& candidates,
                          const geometry::LineString2d& line) {
    if (line.empty()) {
        return;
    }

    for (size_t idx = 0; idx < line.size(); ++idx) {
        appendUniqueCandidate(candidates, line[idx]);
        if (idx + 1 >= line.size()) {
            continue;
        }
        if (geometry::arePointsClose(line[idx], line[idx + 1], kCandidateDedupTolerance)) {
            continue;
        }
        appendUniqueCandidate(
            candidates,
            geometry::Point2d(
                (line[idx].x() + line[idx + 1].x()) * 0.5,
                (line[idx].y() + line[idx + 1].y()) * 0.5));
    }
}

void appendPolygonCandidates(std::vector<geometry::Point2d>& candidates,
                             const geometry::Polygon2d& polygon) {
    const auto& outer = polygon.outer();
    if (outer.size() < 2) {
        return;
    }

    const size_t unique_vertex_count =
        geometry::arePointsClose(outer.front(), outer.back(), kCandidateDedupTolerance)
            ? outer.size() - 1
            : outer.size();
    for (size_t idx = 0; idx < unique_vertex_count; ++idx) {
        appendUniqueCandidate(candidates, outer[idx]);
        const auto& next = outer[(idx + 1) % unique_vertex_count];
        if (geometry::arePointsClose(outer[idx], next, kCandidateDedupTolerance)) {
            continue;
        }
        appendUniqueCandidate(
            candidates,
            geometry::Point2d(
                (outer[idx].x() + next.x()) * 0.5,
                (outer[idx].y() + next.y()) * 0.5));
    }
}

std::vector<geometry::Point2d> deriveHandoffCandidates(
    const geometry::Polygon2d& current_polygon,
    const geometry::Polygon2d& next_polygon) {
    std::vector<geometry::Point2d> candidates;

    MultiPolygon2d overlap;
    geometry::bg::intersection(
        correctedPolygon(current_polygon),
        correctedPolygon(next_polygon),
        overlap);

    double overlap_area = 0.0;
    for (const auto& polygon : overlap) {
        overlap_area += std::abs(geometry::bg::area(polygon));
    }
    if (overlap_area > kOverlapAreaTolerance) {
        for (const auto& polygon : overlap) {
            appendPolygonCandidates(candidates, correctedPolygon(polygon));
        }
        return candidates;
    }

    MultiLineString2d shared_boundary;
    geometry::bg::intersection(
        boundaryLineString(correctedPolygon(current_polygon)),
        boundaryLineString(correctedPolygon(next_polygon)),
        shared_boundary);
    for (const auto& line : shared_boundary) {
        appendLineCandidates(candidates, line);
    }

    return candidates;
}

const SearchFrontierDescriptor* findOwningFrontier(
    const geometry::Point2d& point,
    const ZoneSelectionResult& selection) {
    const SearchFrontierDescriptor* owner = nullptr;
    for (const auto& frontier : selection.frontiers) {
        if (frontier.zone == nullptr) {
            continue;
        }
        if (ZoneSelector::isInsidePolygon(point, frontier.zone->getPolygon())) {
            owner = &frontier;
        }
    }
    return owner;
}

TrackLaneGuidance buildGuidanceForTrackFrontier(
    const SearchFrontierDescriptor& frontier,
    const ZoneSelectionResult& selection,
    const math::Pose2d& goal,
    const std::vector<PolylineSegment>& centerline_segments) {
    const geometry::Point2d goal_point(goal.x, goal.y);
    TrackLaneGuidance guidance;
    guidance.frontier_id = frontier.frontier_id;
    guidance.target_kind = TrackLaneGuidanceTargetKind::Goal;

    const SearchFrontierDescriptor* goal_owner = findOwningFrontier(goal_point, selection);
    geometry::Point2d target_point = goal_point;
    if (goal_owner == nullptr || goal_owner->frontier_id != frontier.frontier_id) {
        guidance.target_kind = TrackLaneGuidanceTargetKind::Handoff;
        if (frontier.frontier_id + 1 < selection.frontiers.size()) {
            const auto& next_frontier = selection.frontiers[frontier.frontier_id + 1];
            if (next_frontier.zone != nullptr) {
                const auto candidates = deriveHandoffCandidates(
                    frontier.zone->getPolygon(),
                    next_frontier.zone->getPolygon());
                if (!candidates.empty()) {
                    target_point = *std::min_element(
                        candidates.begin(),
                        candidates.end(),
                        [&](const auto& lhs, const auto& rhs) {
                            return squaredDistance(lhs, goal_point) <
                                   squaredDistance(rhs, goal_point);
                        });
                }
            }
        }
    }

    const auto target_projection = projectOntoSegments(target_point, centerline_segments);
    if (target_projection.valid) {
        guidance.target_station = target_projection.station_m;
    }
    return guidance;
}

} // namespace

std::vector<TrackLaneGuidance> LaneCenterlineLayer::build(
    grid_map::GridMap& costmap,
    const ZoneSelectionResult& selection,
    const math::Pose2d&,
    const math::Pose2d& goal,
    double max_lane_cost,
    double max_half_width) {

    costmap.add(CostmapLayerNames::LANE_CENTERLINE_COST, 0.0f);
    costmap.add(
        CostmapLayerNames::LANE_HEADING,
        std::numeric_limits<float>::quiet_NaN());
    costmap.add(
        CostmapLayerNames::LANE_DISTANCE,
        std::numeric_limits<float>::infinity());
    costmap.add(
        CostmapLayerNames::TRACK_STATION,
        std::numeric_limits<float>::quiet_NaN());

    std::vector<TrackLaneGuidance> guidance;
    std::vector<TrackInfo> tracks;
    for (const auto& frontier : selection.frontiers) {
        auto track = std::dynamic_pointer_cast<zones::TrackMainRoad>(frontier.zone);
        if (!track) {
            continue;
        }

        const auto centerline_segments = buildPolylineSegments(track->getCenterline());
        const auto& lanes = track->getLanes();
        if (centerline_segments.empty() || lanes.size() < 2) {
            continue;
        }

        TrackInfo info;
        info.frontier_id = frontier.frontier_id;
        info.track = std::move(track);
        info.centerline_segments = centerline_segments;
        info.lane_segments[0] = buildPolylineSegments(lanes[0]);
        info.lane_segments[1] = buildPolylineSegments(lanes[1]);
        if (info.lane_segments[0].empty() || info.lane_segments[1].empty()) {
            continue;
        }

        const auto track_guidance =
            buildGuidanceForTrackFrontier(frontier, selection, goal, info.centerline_segments);
        info.target_station_m = track_guidance.target_station;
        info.target_kind = track_guidance.target_kind;
        info.gm_polygon = buildGridMapPolygon(info.track->getPolygon());

        guidance.push_back(track_guidance);
        tracks.push_back(std::move(info));
    }

    for (const auto& info : tracks) {
        for (grid_map::PolygonIterator it(costmap, info.gm_polygon);
             !it.isPastEnd(); ++it) {
            const float frontier_owner =
                costmap.at(CostmapLayerNames::ZONE_CONSTRAINTS, *it);
            if (std::isnan(frontier_owner) ||
                std::abs(frontier_owner - static_cast<float>(info.frontier_id)) >= 0.5f) {
                continue;
            }

            grid_map::Position pos;
            costmap.getPosition(*it, pos);

            const geometry::Point2d cell_point(pos.x(), pos.y());
            const auto center_projection =
                projectOntoSegments(cell_point, info.centerline_segments);
            if (!center_projection.valid) {
                continue;
            }

            costmap.at(CostmapLayerNames::TRACK_STATION, *it) =
                static_cast<float>(center_projection.station_m);

            const size_t preferred_lane_idx =
                info.target_station_m >= center_projection.station_m ? 0u : 1u;
            const auto lane_projection =
                projectOntoSegments(cell_point, info.lane_segments[preferred_lane_idx]);
            if (!lane_projection.valid) {
                continue;
            }

            const double preferred_lane_distance = std::sqrt(lane_projection.distance_sq);
            const double normalized =
                max_half_width > 0.0
                    ? std::min(preferred_lane_distance / max_half_width, 1.0)
                    : 0.0;

            costmap.at(CostmapLayerNames::LANE_CENTERLINE_COST, *it) =
                static_cast<float>(normalized * max_lane_cost);
            costmap.at(CostmapLayerNames::LANE_HEADING, *it) =
                static_cast<float>(lane_projection.heading_rad);
            costmap.at(CostmapLayerNames::LANE_DISTANCE, *it) =
                static_cast<float>(preferred_lane_distance);
        }
    }

    return guidance;
}

} // namespace costs
} // namespace coastmotionplanning
