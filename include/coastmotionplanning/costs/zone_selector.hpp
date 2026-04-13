#pragma once

#include <memory>
#include <vector>

#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/geometry/shape_types.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace costs {

/// Result of zone selection for a planning query
struct ZoneSelectionResult {
    std::vector<std::shared_ptr<zones::Zone>> selected_zones;
    geometry::Polygon2d search_boundary;  // Concave hull encompassing selected zones
};

/// Selects the relevant zones for a planning query and computes a concave hull
/// search boundary that tightly wraps those zones (bridging any gaps).
class ZoneSelector {
public:
    ZoneSelector() = default;

    /// Select the zones containing start and goal, then compute the search boundary.
    /// @param start       Start pose
    /// @param goal        Goal pose
    /// @param all_zones   All parsed zones from the map
    /// @param alpha       Alpha-shape parameter (0 = auto from typical zone spacing)
    /// @return ZoneSelectionResult with selected zones and search boundary polygon
    /// @throws std::runtime_error if start or goal is not inside any zone
    ZoneSelectionResult select(
        const math::Pose2d& start,
        const math::Pose2d& goal,
        const std::vector<std::shared_ptr<zones::Zone>>& all_zones,
        double alpha = 0.0) const;

    /// Determine which zone a position falls inside.
    /// @return shared_ptr to the zone, or nullptr if not inside any zone
    static std::shared_ptr<zones::Zone> findContainingZone(
        const geometry::Point2d& point,
        const std::vector<std::shared_ptr<zones::Zone>>& zones);

    /// Compute a concave hull (alpha-shape) around a set of zone polygons.
    /// This bridges gaps between non-adjacent zones while keeping a tight boundary.
    /// @param zone_polygons  The polygons to wrap
    /// @param alpha          Alpha parameter (smaller = tighter, 0 = auto)
    /// @return The concave hull polygon
    static geometry::Polygon2d computeConcaveHull(
        const std::vector<geometry::Polygon2d>& zone_polygons,
        double alpha);

    /// Simple check if a point is inside a polygon.
    static bool isInsidePolygon(
        const geometry::Point2d& point,
        const geometry::Polygon2d& polygon);
};

} // namespace costs
} // namespace coastmotionplanning
