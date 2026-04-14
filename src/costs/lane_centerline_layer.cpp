#include "coastmotionplanning/costs/lane_centerline_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "coastmotionplanning/zones/track_main_road.hpp"

namespace coastmotionplanning {
namespace costs {

namespace {

struct LaneSegment {
    geometry::Point2d start;
    geometry::Point2d end;
    double heading_rad{0.0};
    double length_sq{0.0};
};

double distanceSquaredToSegment(const geometry::Point2d& point,
                                const LaneSegment& segment) {
    const double px = geometry::bg::get<0>(point);
    const double py = geometry::bg::get<1>(point);
    const double x0 = geometry::bg::get<0>(segment.start);
    const double y0 = geometry::bg::get<1>(segment.start);
    const double x1 = geometry::bg::get<0>(segment.end);
    const double y1 = geometry::bg::get<1>(segment.end);

    if (segment.length_sq <= 1e-12) {
        const double dx = px - x0;
        const double dy = py - y0;
        return (dx * dx) + (dy * dy);
    }

    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double t = std::clamp(
        (((px - x0) * dx) + ((py - y0) * dy)) / segment.length_sq,
        0.0,
        1.0);
    const double nearest_x = x0 + (t * dx);
    const double nearest_y = y0 + (t * dy);
    const double dist_x = px - nearest_x;
    const double dist_y = py - nearest_y;
    return (dist_x * dist_x) + (dist_y * dist_y);
}

std::vector<LaneSegment> buildLaneSegments(
    const std::vector<std::vector<math::Pose2d>>& lanes) {
    std::vector<LaneSegment> segments;
    for (const auto& lane : lanes) {
        for (size_t idx = 0; idx + 1 < lane.size(); ++idx) {
            const geometry::Point2d start(lane[idx].x, lane[idx].y);
            const geometry::Point2d end(lane[idx + 1].x, lane[idx + 1].y);
            const double dx = end.x() - start.x();
            const double dy = end.y() - start.y();
            const double length_sq = (dx * dx) + (dy * dy);
            if (length_sq <= 1e-12) {
                continue;
            }

            LaneSegment segment;
            segment.start = start;
            segment.end = end;
            segment.heading_rad = std::atan2(dy, dx);
            segment.length_sq = length_sq;
            segments.push_back(segment);
        }
    }
    return segments;
}

} // namespace

void LaneCenterlineLayer::build(
    grid_map::GridMap& costmap,
    const std::vector<std::shared_ptr<zones::Zone>>& selected_zones,
    double max_lane_cost,
    double max_half_width) {

    // Add layer initialized to 0 (free / no lane preference)
    costmap.add(CostmapLayerNames::LANE_CENTERLINE_COST, 0.0f);
    costmap.add(
        CostmapLayerNames::LANE_HEADING,
        std::numeric_limits<float>::quiet_NaN());
    costmap.add(
        CostmapLayerNames::LANE_DISTANCE,
        std::numeric_limits<float>::infinity());

    // Collect all TrackMainRoad zones and their lane centerlines
    struct TrackInfo {
        std::shared_ptr<zones::TrackMainRoad> track;
        std::vector<LaneSegment> segments;
        grid_map::Polygon gm_polygon;
    };

    std::vector<TrackInfo> tracks;
    for (const auto& z : selected_zones) {
        auto track = std::dynamic_pointer_cast<zones::TrackMainRoad>(z);
        if (!track) continue;

        TrackInfo info;
        info.track = track;
        info.segments = buildLaneSegments(track->getLanes());
        if (info.segments.empty()) continue;

        // Build grid_map polygon for iteration
        const auto& outer = track->getPolygon().outer();
        for (size_t i = 0; i < outer.size(); ++i) {
            if (i == outer.size() - 1) {
                const auto& first = outer.front();
                const auto& last = outer.back();
                if (std::abs(geometry::bg::get<0>(first) - geometry::bg::get<0>(last)) < 1e-9 &&
                    std::abs(geometry::bg::get<1>(first) - geometry::bg::get<1>(last)) < 1e-9) {
                    continue;
                }
            }
            info.gm_polygon.addVertex(grid_map::Position(
                geometry::bg::get<0>(outer[i]),
                geometry::bg::get<1>(outer[i])));
        }

        tracks.push_back(std::move(info));
    }

    // For each track zone, compute distance-to-centerline cost
    for (const auto& info : tracks) {
        for (grid_map::PolygonIterator it(costmap, info.gm_polygon);
             !it.isPastEnd(); ++it) {
            grid_map::Position pos;
            costmap.getPosition(*it, pos);

            geometry::Point2d cell_pt(pos.x(), pos.y());
            double min_dist_sq = std::numeric_limits<double>::infinity();
            double nearest_heading = std::numeric_limits<double>::quiet_NaN();
            for (const auto& segment : info.segments) {
                const double dist_sq = distanceSquaredToSegment(cell_pt, segment);
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    nearest_heading = segment.heading_rad;
                }
            }
            const double min_dist = std::sqrt(min_dist_sq);

            // Normalize: 0 at centerline, max_lane_cost at max_half_width
            const double normalized =
                max_half_width > 0.0
                    ? std::min(min_dist / max_half_width, 1.0)
                    : 0.0;
            float cost = static_cast<float>(normalized * max_lane_cost);

            costmap.at(CostmapLayerNames::LANE_CENTERLINE_COST, *it) = cost;
            costmap.at(CostmapLayerNames::LANE_HEADING, *it) =
                static_cast<float>(nearest_heading);
            costmap.at(CostmapLayerNames::LANE_DISTANCE, *it) =
                static_cast<float>(min_dist);
        }
    }
}

} // namespace costs
} // namespace coastmotionplanning
