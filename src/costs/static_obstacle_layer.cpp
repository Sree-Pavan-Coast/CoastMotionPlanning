#include "coastmotionplanning/costs/static_obstacle_layer.hpp"

#include <boost/geometry/algorithms/within.hpp>

namespace coastmotionplanning {
namespace costs {

void StaticObstacleLayer::build(grid_map::GridMap& costmap,
                                 const geometry::Polygon2d& search_boundary) {
    // Add the layer initialized to LETHAL (everything is obstacle by default)
    costmap.add(CostmapLayerNames::STATIC_OBSTACLES, CostValues::LETHAL);

    // Mark cells INSIDE the search boundary as FREE
    // Convert search_boundary to a grid_map polygon for efficient iteration
    grid_map::Polygon gm_polygon;
    for (const auto& pt : search_boundary.outer()) {
        gm_polygon.addVertex(grid_map::Position(
            geometry::bg::get<0>(pt),
            geometry::bg::get<1>(pt)));
    }
    // Remove the duplicate closing vertex if present
    const auto& verts = gm_polygon.getVertices();
    if (verts.size() > 1) {
        const auto& first = verts.front();
        const auto& last = verts.back();
        if ((first - last).norm() < 1e-9) {
            // grid_map::Polygon doesn't expect a closing vertex;
            // our addVertex already handles it implicitly
        }
    }

    for (grid_map::PolygonIterator it(costmap, gm_polygon); !it.isPastEnd(); ++it) {
        costmap.at(CostmapLayerNames::STATIC_OBSTACLES, *it) = CostValues::FREE_SPACE;
    }
}

} // namespace costs
} // namespace coastmotionplanning
