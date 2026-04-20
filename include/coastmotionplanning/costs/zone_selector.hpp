#pragma once

#include <memory>
#include <string>
#include <vector>

#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/geometry/shape_types.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace costs {

enum class SearchFrontierRole {
    StartZone,
    GoalZone
};

struct ZoneConnectivityIndex {
    std::vector<std::vector<size_t>> adjacent_zone_indices;

    bool areDirectlyConnected(size_t first_index, size_t second_index) const;
    bool isIsolated(size_t zone_index) const;
};

struct SearchFrontierDescriptor {
    size_t frontier_id{0};
    SearchFrontierRole role{SearchFrontierRole::StartZone};
    std::shared_ptr<zones::Zone> zone;
    std::string behavior_name;
};

/// Result of zone selection for a planning query
struct ZoneSelectionResult {
    std::vector<std::shared_ptr<zones::Zone>> selected_zones;
    std::vector<SearchFrontierDescriptor> frontiers;
    geometry::Polygon2d search_boundary;  // Tight connected boundary over selected zones
};

/// Selects the relevant zones for a planning query and computes the exact
/// connected search boundary from authored zone geometry only.
class ZoneSelector {
public:
    ZoneSelector() = default;

    /// Select the zones containing start and goal, then compute the exact search boundary.
    /// @param start       Start pose
    /// @param goal        Goal pose
    /// @param all_zones   All parsed zones from the map
    /// @return ZoneSelectionResult with selected zones and search boundary polygon
    /// @throws std::runtime_error if start or goal is not inside any zone
    ZoneSelectionResult select(
        const math::Pose2d& start,
        const math::Pose2d& goal,
        const std::vector<std::shared_ptr<zones::Zone>>& all_zones,
        const std::string& start_frontier_behavior = "") const;

    /// Build direct-connectivity metadata for a map's authored zones.
    /// Two zones are directly connected when they share an edge or overlap in area.
    static ZoneConnectivityIndex buildConnectivityIndex(
        const std::vector<std::shared_ptr<zones::Zone>>& zones);

    /// Determine which zone a position falls inside.
    /// @return shared_ptr to the zone, or nullptr if not inside any zone
    static std::shared_ptr<zones::Zone> findContainingZone(
        const geometry::Point2d& point,
        const std::vector<std::shared_ptr<zones::Zone>>& zones);

    /// Simple check if a point is inside a polygon.
    static bool isInsidePolygon(
        const geometry::Point2d& point,
        const geometry::Polygon2d& polygon);
};

} // namespace costs
} // namespace coastmotionplanning
