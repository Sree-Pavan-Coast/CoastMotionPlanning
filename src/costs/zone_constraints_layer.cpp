#include "coastmotionplanning/costs/zone_constraints_layer.hpp"

#include <cmath>

namespace coastmotionplanning {
namespace costs {

void ZoneConstraintsLayer::build(
    grid_map::GridMap& costmap,
    const ZoneSelectionResult& selection) {

    // Add layer initialized to ZONE_NONE (lethal for anything outside zones)
    costmap.add(CostmapLayerNames::ZONE_CONSTRAINTS, ZoneConstraintValues::ZONE_NONE);

    // Stamp concrete selected-zone polygons in frontier order.
    // Later frontiers overwrite earlier ones so overlap cells belong to the
    // goal frontier for cross-zone planning.
    for (const auto& frontier : selection.frontiers) {
        if (frontier.zone == nullptr) {
            continue;
        }
        const auto& zone = frontier.zone;

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

        const float frontier_value = static_cast<float>(frontier.frontier_id);
        for (grid_map::PolygonIterator it(costmap, gm_polygon);
             !it.isPastEnd(); ++it) {
            costmap.at(CostmapLayerNames::ZONE_CONSTRAINTS, *it) = frontier_value;
        }
    }
}

} // namespace costs
} // namespace coastmotionplanning
