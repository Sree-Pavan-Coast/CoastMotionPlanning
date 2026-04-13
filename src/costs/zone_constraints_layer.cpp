#include "coastmotionplanning/costs/zone_constraints_layer.hpp"

#include <boost/geometry/algorithms/within.hpp>

namespace coastmotionplanning {
namespace costs {

void ZoneConstraintsLayer::build(
    grid_map::GridMap& costmap,
    const std::vector<std::shared_ptr<zones::Zone>>& selected_zones,
    const geometry::Polygon2d& search_boundary) {

    (void)search_boundary;  // Reserved for future use

    // Add layer initialized to ZONE_NONE (lethal for anything outside zones)
    costmap.add(CostmapLayerNames::ZONE_CONSTRAINTS, ZoneConstraintValues::ZONE_NONE);

    // For each zone, stamp cells with the zone's index (0-based).
    // The planner can look up the zone object by this index to query
    // isReverseAllowed(), getActiveLayers(), etc.
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
            costmap.at(CostmapLayerNames::ZONE_CONSTRAINTS, *it) = zone_value;
        }
    }
}

} // namespace costs
} // namespace coastmotionplanning
