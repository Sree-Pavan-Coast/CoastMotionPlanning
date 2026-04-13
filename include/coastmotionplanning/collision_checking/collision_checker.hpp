#pragma once

#include <string>
#include <vector>
#include <grid_map_core/grid_map_core.hpp>
#include "coastmotionplanning/geometry/shape_types.hpp"
#include "coastmotionplanning/robot/robot_base.hpp"

namespace coastmotionplanning {
namespace collision_checking {

/// Result of a collision query with diagnostic info
struct CollisionResult {
    bool in_collision{false};
    float max_cost{0.0f};             // Highest cost cell found under the footprint
    grid_map::Position worst_cell;     // World position of the highest-cost cell
};

/// Configuration for the collision checker
struct CollisionCheckerConfig {
    std::string obstacle_layer{"obstacle_cost"};  // Layer name to check against
    float lethal_threshold{254.0f};               // Cells at or above this are lethal
    float inscribed_threshold{128.0f};             // Cells at or above this are inscribed (soft collision)
};

/// Checks a robot footprint polygon against a grid_map costmap layer.
///
/// This class bridges the gap between the robot model (which produces
/// Boost.Geometry footprint polygons) and the grid_map costmap (which
/// stores per-cell cost values). It rasterizes the footprint onto the
/// grid using grid_map's PolygonIterator and checks for lethal cells.
class CollisionChecker {
public:
    explicit CollisionChecker(const CollisionCheckerConfig& config = {});
    ~CollisionChecker() = default;

    /// Check if a robot footprint at a given state collides with obstacles in the costmap.
    /// @param costmap  The grid_map containing cost layers
    /// @param robot    The robot model (provides footprint polygon)
    /// @param state    The robot state (x, y, yaw, articulations)
    /// @return CollisionResult with collision status and diagnostics
    CollisionResult checkCollision(
        const grid_map::GridMap& costmap,
        const robot::RobotBase& robot,
        const robot::RobotState& state) const;

    /// Check if a pre-computed footprint polygon collides with obstacles in the costmap.
    /// Use this overload when you've already computed the footprint (avoids recomputation).
    /// @param costmap   The grid_map containing cost layers
    /// @param footprint The polygon to check (in world coordinates)
    /// @return CollisionResult with collision status and diagnostics
    CollisionResult checkFootprint(
        const grid_map::GridMap& costmap,
        const geometry::Polygon2d& footprint) const;

    /// Compute the maximum cost under a footprint without applying the lethal threshold.
    /// Useful for cost-aware planning where you want to penalize nearness to obstacles
    /// without a hard binary collision check.
    /// @param costmap   The grid_map containing cost layers
    /// @param footprint The polygon to check (in world coordinates)
    /// @return The maximum cost value found under the footprint (0 if footprint is outside map)
    float getFootprintCost(
        const grid_map::GridMap& costmap,
        const geometry::Polygon2d& footprint) const;

    /// Check if a single point is in collision (lethal cell).
    /// @param costmap  The grid_map containing cost layers
    /// @param position World position to check
    /// @return true if the cell at position is at or above the lethal threshold
    bool isPointInCollision(
        const grid_map::GridMap& costmap,
        const grid_map::Position& position) const;

    // Accessors
    const CollisionCheckerConfig& getConfig() const { return config_; }
    void setConfig(const CollisionCheckerConfig& config) { config_ = config; }

private:
    /// Convert a Boost.Geometry Polygon2d to a grid_map::Polygon for use with iterators
    grid_map::Polygon toGridMapPolygon(const geometry::Polygon2d& boost_polygon) const;

    CollisionCheckerConfig config_;
};

} // namespace collision_checking
} // namespace coastmotionplanning
