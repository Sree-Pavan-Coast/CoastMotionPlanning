#include "coastmotionplanning/costs/zone_constraints_layer.hpp"

#include <algorithm>
#include <cmath>

#include <boost/geometry/algorithms/within.hpp>

namespace coastmotionplanning {
namespace costs {

void ZoneConstraintsLayer::build(
    grid_map::GridMap& costmap,
    const ZoneSelectionResult& selection) {

    // Add layer initialized to ZONE_NONE (lethal for anything outside zones)
    costmap.add(CostmapLayerNames::ZONE_CONSTRAINTS, ZoneConstraintValues::ZONE_NONE);

    // Stamp the search boundary with the transition frontier when one exists.
    // This makes the handoff frontier first-class instead of using a generic
    // "keep current behavior" transition sentinel.
    const auto transition_it = std::find_if(
        selection.frontiers.begin(),
        selection.frontiers.end(),
        [](const SearchFrontierDescriptor& frontier) {
            return frontier.role == SearchFrontierRole::Transition;
        });
    {
        grid_map::Polygon gm_boundary;
        for (const auto& pt : selection.search_boundary.outer()) {
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
                transition_it != selection.frontiers.end()
                    ? static_cast<float>(transition_it->frontier_id)
                    : ZoneConstraintValues::ZONE_NONE;
        }
    }

    // Stamp all frontier-owned concrete zone polygons. Concrete frontiers
    // overwrite the transition frontier, but not previously stamped concrete
    // frontiers, which preserves ordering when polygons overlap.
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
            float& cell = costmap.at(CostmapLayerNames::ZONE_CONSTRAINTS, *it);
            if (cell >= ZoneConstraintValues::ZONE_NONE ||
                transition_it != selection.frontiers.end()) {
                if (cell >= ZoneConstraintValues::ZONE_NONE ||
                    (transition_it != selection.frontiers.end() &&
                     std::abs(cell - static_cast<float>(transition_it->frontier_id)) < 0.5f)) {
                    cell = frontier_value;
                }
            }
        }
    }
}

} // namespace costs
} // namespace coastmotionplanning
