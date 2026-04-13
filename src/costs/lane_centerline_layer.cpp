#include "coastmotionplanning/costs/lane_centerline_layer.hpp"

#include <algorithm>
#include <cmath>

#include <boost/geometry/algorithms/distance.hpp>
#include "coastmotionplanning/zones/track_main_road.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"

namespace coastmotionplanning {
namespace costs {

void LaneCenterlineLayer::build(
    grid_map::GridMap& costmap,
    const std::vector<std::shared_ptr<zones::Zone>>& selected_zones,
    double max_lane_cost,
    double max_half_width) {

    // Add layer initialized to 0 (free / no lane preference)
    costmap.add(CostmapLayerNames::LANE_CENTERLINE_COST, 0.0f);

    // Collect all TrackMainRoad zones and their lane centerlines
    struct TrackInfo {
        std::shared_ptr<zones::TrackMainRoad> track;
        geometry::LineString2d centerline;
        grid_map::Polygon gm_polygon;
    };

    std::vector<TrackInfo> tracks;
    for (const auto& z : selected_zones) {
        auto track = std::dynamic_pointer_cast<zones::TrackMainRoad>(z);
        if (!track) continue;

        const auto& waypoints = track->getLaneWaypoints();
        if (waypoints.size() < 2) continue;

        TrackInfo info;
        info.track = track;

        // Build LineString from waypoints
        for (const auto& wp : waypoints) {
            info.centerline.push_back(geometry::Point2d(wp.x, wp.y));
        }

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
            double dist = geometry::bg::distance(cell_pt, info.centerline);

            // Normalize: 0 at centerline, max_lane_cost at max_half_width
            double normalized = std::min(dist / max_half_width, 1.0);
            float cost = static_cast<float>(normalized * max_lane_cost);

            costmap.at(CostmapLayerNames::LANE_CENTERLINE_COST, *it) = cost;
        }
    }
}

} // namespace costs
} // namespace coastmotionplanning
