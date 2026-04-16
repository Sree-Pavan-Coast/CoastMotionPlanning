#include "coastmotionplanning/costs/static_obstacle_layer.hpp"

#include <cmath>

namespace coastmotionplanning {
namespace costs {

namespace {

void stampPolygon(grid_map::GridMap& costmap,
                  const geometry::Polygon2d& polygon,
                  const float fill_value) {
    geometry::Polygon2d corrected_polygon = polygon;
    geometry::bg::correct(corrected_polygon);
    if (corrected_polygon.outer().size() < 4 ||
        std::abs(geometry::bg::area(corrected_polygon)) < 1e-9) {
        return;
    }

    grid_map::Polygon gm_polygon;
    const auto& outer = corrected_polygon.outer();
    const size_t vertex_count =
        geometry::arePointsClose(outer.front(), outer.back())
            ? outer.size() - 1
            : outer.size();
    if (vertex_count < 3) {
        return;
    }

    for (size_t i = 0; i < vertex_count; ++i) {
        gm_polygon.addVertex(grid_map::Position(
            geometry::bg::get<0>(outer[i]),
            geometry::bg::get<1>(outer[i])));
    }

    for (grid_map::PolygonIterator it(costmap, gm_polygon); !it.isPastEnd(); ++it) {
        costmap.at(CostmapLayerNames::STATIC_OBSTACLES, *it) = fill_value;
    }
}

} // namespace

void StaticObstacleLayer::build(grid_map::GridMap& costmap,
                                const geometry::Polygon2d& search_boundary,
                                const std::vector<geometry::Polygon2d>& obstacle_polygons) {
    // Add the layer initialized to LETHAL (everything is obstacle by default)
    costmap.add(CostmapLayerNames::STATIC_OBSTACLES, CostValues::LETHAL);

    // Clear the operational area to FREE, then stamp runtime obstacle polygons back to LETHAL.
    stampPolygon(costmap, search_boundary, CostValues::FREE_SPACE);
    for (const auto& obstacle_polygon : obstacle_polygons) {
        stampPolygon(costmap, obstacle_polygon, CostValues::LETHAL);
    }
}

} // namespace costs
} // namespace coastmotionplanning
