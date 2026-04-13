#include "coastmotionplanning/collision_checking/collision_checker.hpp"

#include <limits>

namespace coastmotionplanning {
namespace collision_checking {

CollisionChecker::CollisionChecker(const CollisionCheckerConfig& config)
    : config_(config) {}

CollisionResult CollisionChecker::checkCollision(
    const grid_map::GridMap& costmap,
    const robot::RobotBase& robot,
    const robot::RobotState& state) const {
    // Compute the world-frame footprint from the robot model and state
    geometry::Polygon2d footprint = robot.getRobotFootprint(state);
    return checkFootprint(costmap, footprint);
}

CollisionResult CollisionChecker::checkFootprint(
    const grid_map::GridMap& costmap,
    const geometry::Polygon2d& footprint) const {
    CollisionResult result;
    result.max_cost = 0.0f;
    result.in_collision = false;

    // Bail early if the layer doesn't exist
    if (!costmap.exists(config_.obstacle_layer)) {
        return result;
    }

    // Convert boost polygon to grid_map polygon
    grid_map::Polygon gm_polygon = toGridMapPolygon(footprint);

    // Iterate over all cells covered by the footprint
    for (grid_map::PolygonIterator iterator(costmap, gm_polygon);
         !iterator.isPastEnd(); ++iterator) {
        const float cell_cost = costmap.at(config_.obstacle_layer, *iterator);

        // Skip NaN cells (unknown/uninitialized)
        if (std::isnan(cell_cost)) {
            continue;
        }

        if (cell_cost > result.max_cost) {
            result.max_cost = cell_cost;
            costmap.getPosition(*iterator, result.worst_cell);
        }

        if (cell_cost >= config_.lethal_threshold) {
            result.in_collision = true;
            // Early exit: we already know it's a collision.
            // We still have the worst_cell from the lethal cell we just found.
            return result;
        }
    }

    return result;
}

float CollisionChecker::getFootprintCost(
    const grid_map::GridMap& costmap,
    const geometry::Polygon2d& footprint) const {
    if (!costmap.exists(config_.obstacle_layer)) {
        return 0.0f;
    }

    grid_map::Polygon gm_polygon = toGridMapPolygon(footprint);
    float max_cost = 0.0f;

    for (grid_map::PolygonIterator iterator(costmap, gm_polygon);
         !iterator.isPastEnd(); ++iterator) {
        const float cell_cost = costmap.at(config_.obstacle_layer, *iterator);
        if (!std::isnan(cell_cost) && cell_cost > max_cost) {
            max_cost = cell_cost;
        }
    }

    return max_cost;
}

bool CollisionChecker::isPointInCollision(
    const grid_map::GridMap& costmap,
    const grid_map::Position& position) const {
    if (!costmap.exists(config_.obstacle_layer)) {
        return false;
    }

    if (!costmap.isInside(position)) {
        // Position is outside the map — treat as collision (conservative)
        return true;
    }

    const float cost = costmap.atPosition(config_.obstacle_layer, position);
    if (std::isnan(cost)) {
        return false; // Unknown cells are treated as free
    }

    return cost >= config_.lethal_threshold;
}

grid_map::Polygon CollisionChecker::toGridMapPolygon(
    const geometry::Polygon2d& boost_polygon) const {
    grid_map::Polygon gm_polygon;

    // Boost.Geometry polygons store vertices in outer().
    // The last vertex is typically a duplicate of the first (closed ring),
    // but grid_map::Polygon does not expect a closing vertex, so we skip it.
    const auto& outer = boost_polygon.outer();
    if (outer.empty()) {
        return gm_polygon;
    }

    // Skip the last vertex if it's a duplicate of the first (closed ring)
    size_t count = outer.size();
    if (count > 1) {
        const auto& first = outer.front();
        const auto& last = outer.back();
        if (geometry::bg::get<0>(first) == geometry::bg::get<0>(last) &&
            geometry::bg::get<1>(first) == geometry::bg::get<1>(last)) {
            count -= 1;
        }
    }

    for (size_t i = 0; i < count; ++i) {
        gm_polygon.addVertex(grid_map::Position(
            geometry::bg::get<0>(outer[i]),
            geometry::bg::get<1>(outer[i])
        ));
    }

    return gm_polygon;
}

} // namespace collision_checking
} // namespace coastmotionplanning
