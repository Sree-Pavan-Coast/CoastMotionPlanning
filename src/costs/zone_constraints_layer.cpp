#include "coastmotionplanning/costs/zone_constraints_layer.hpp"

#include <boost/geometry/algorithms/within.hpp>

namespace coastmotionplanning {
namespace costs {

void ZoneConstraintsLayer::build(
    grid_map::GridMap& costmap,
    const std::vector<std::shared_ptr<zones::Zone>>& selected_zones,
    const geometry::Polygon2d& search_boundary) {

    // Add layer initialized to ZONE_NONE (lethal for anything outside zones)
    costmap.add(CostmapLayerNames::ZONE_CONSTRAINTS, ZoneConstraintValues::ZONE_NONE);

    // Mark cells inside the search boundary but outside any zone as ZONE_TRANSITION
    // so the planner can traverse gap regions between zones.
    {
        grid_map::Polygon gm_boundary;
        for (const auto& pt : search_boundary.outer()) {
            gm_boundary.addVertex(grid_map::Position(
                geometry::bg::get<0>(pt), geometry::bg::get<1>(pt)));
        }
        // Remove closing vertex if present (grid_map::Polygon closes implicitly)
        const auto& verts = gm_boundary.getVertices();
        if (verts.size() > 1 && (verts.front() - verts.back()).norm() < 1e-9) {
            // grid_map::Polygon doesn't provide removeVertex, but the closing
            // duplicate is harmless for PolygonIterator — leave it.
        }
        for (grid_map::PolygonIterator it(costmap, gm_boundary);
             !it.isPastEnd(); ++it) {
            costmap.at(CostmapLayerNames::ZONE_CONSTRAINTS, *it) =
                ZoneConstraintValues::ZONE_TRANSITION;
        }
    }

    // For each zone, stamp cells with the zone's index (0-based).
    // Zones overwrite transition cells, but not previously stamped zone cells,
    // which preserves first-zone ordering when polygons overlap.
    for (size_t zone_idx = 0; zone_idx < selected_zones.size(); ++zone_idx) {
        const auto& zone = selected_zones[zone_idx];

        // Convert to grid_map polygon
        grid_map::Polygon gm_polygon;
        const auto& outer = zone->getPolygon().outer();
        for (size_t i = 0; i < outer.size(); ++i) {
            // Skip closing vertex
            if (i == outer.size() - 1) {
                const auto& first = outer.front();
                const auto& last = outer.back();
                if (std::abs(geometry::bg::get<0>(first) - geometry::bg::get<0>(last)) < 1e-9 &&
                    std::abs(geometry::bg::get<1>(first) - geometry::bg::get<1>(last)) < 1e-9) {
                    continue;
                }
            }
            gm_polygon.addVertex(grid_map::Position(
                geometry::bg::get<0>(outer[i]),
                geometry::bg::get<1>(outer[i])));
        }

        float zone_value = static_cast<float>(zone_idx);
        for (grid_map::PolygonIterator it(costmap, gm_polygon);
             !it.isPastEnd(); ++it) {
            float& cell = costmap.at(CostmapLayerNames::ZONE_CONSTRAINTS, *it);
            if (cell >= ZoneConstraintValues::ZONE_TRANSITION) {
                cell = zone_value;
            }
        }
    }
}

} // namespace costs
} // namespace coastmotionplanning
