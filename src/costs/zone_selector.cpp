#include "coastmotionplanning/costs/zone_selector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/length.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

namespace coastmotionplanning {
namespace costs {

namespace {

using MultiPolygon2d = geometry::bg::model::multi_polygon<geometry::Polygon2d>;
using MultiLineString2d = geometry::bg::model::multi_linestring<geometry::LineString2d>;

constexpr double kDegenerateAreaTolerance = 1e-9;

geometry::Polygon2d correctedPolygon(geometry::Polygon2d polygon) {
    geometry::bg::correct(polygon);
    return polygon;
}

geometry::LineString2d boundaryLineString(const geometry::Polygon2d& polygon) {
    geometry::LineString2d boundary;
    const auto& outer = polygon.outer();
    if (outer.empty()) {
        return boundary;
    }
    for (const auto& point : outer) {
        boundary.push_back(point);
    }
    if (!geometry::arePointsClose(boundary.front(), boundary.back())) {
        boundary.push_back(boundary.front());
    }
    return boundary;
}

bool isPolygonDegenerate(const geometry::Polygon2d& polygon) {
    return polygon.outer().size() < 4 ||
           std::abs(geometry::bg::area(polygon)) <= kDegenerateAreaTolerance;
}

double overlapAreaMagnitude(const geometry::Polygon2d& first,
                            const geometry::Polygon2d& second) {
    MultiPolygon2d overlap;
    geometry::bg::intersection(first, second, overlap);

    double area = 0.0;
    for (const auto& polygon : overlap) {
        area += std::abs(geometry::bg::area(polygon));
    }
    return area;
}

double sharedBoundaryLength(const geometry::Polygon2d& first,
                            const geometry::Polygon2d& second) {
    MultiLineString2d shared_segments;
    geometry::bg::intersection(
        boundaryLineString(first),
        boundaryLineString(second),
        shared_segments);

    double total_length = 0.0;
    for (const auto& segment : shared_segments) {
        total_length += geometry::bg::length(segment);
    }
    return total_length;
}

bool arePolygonsDirectlyConnected(const geometry::Polygon2d& first,
                                  const geometry::Polygon2d& second) {
    const geometry::Polygon2d corrected_first = correctedPolygon(first);
    const geometry::Polygon2d corrected_second = correctedPolygon(second);
    if (isPolygonDegenerate(corrected_first) || isPolygonDegenerate(corrected_second)) {
        return false;
    }

    if (overlapAreaMagnitude(corrected_first, corrected_second) > kDegenerateAreaTolerance) {
        return true;
    }

    return sharedBoundaryLength(corrected_first, corrected_second) > common::EPSILON;
}

geometry::Polygon2d exactUnionBoundary(const geometry::Polygon2d& first,
                                       const geometry::Polygon2d& second) {
    MultiPolygon2d merged;
    geometry::bg::union_(correctedPolygon(first), correctedPolygon(second), merged);
    if (merged.size() != 1) {
        throw std::runtime_error(
            "ZoneSelector: directly connected start and goal zones did not produce a single union polygon");
    }
    return correctedPolygon(merged.front());
}

} // anonymous namespace

// =============================================================================
// ZoneSelector implementation
// =============================================================================

bool ZoneConnectivityIndex::areDirectlyConnected(size_t first_index, size_t second_index) const {
    if (first_index == second_index) {
        return true;
    }
    if (first_index >= adjacent_zone_indices.size() ||
        second_index >= adjacent_zone_indices.size()) {
        return false;
    }
    const auto& neighbors = adjacent_zone_indices[first_index];
    return std::find(neighbors.begin(), neighbors.end(), second_index) != neighbors.end();
}

bool ZoneConnectivityIndex::isIsolated(size_t zone_index) const {
    return zone_index >= adjacent_zone_indices.size() ||
           adjacent_zone_indices[zone_index].empty();
}

ZoneSelectionResult ZoneSelector::select(
    const math::Pose2d& start,
    const math::Pose2d& goal,
    const std::vector<std::shared_ptr<zones::Zone>>& all_zones,
    const std::string& start_frontier_behavior) const {
    geometry::Point2d start_pt(start.x, start.y);
    geometry::Point2d goal_pt(goal.x, goal.y);

    const auto start_zone = findContainingZone(start_pt, all_zones);
    const auto goal_zone = findContainingZone(goal_pt, all_zones);

    if (!start_zone) {
        throw std::runtime_error("ZoneSelector: start position is not inside any zone");
    }
    if (!goal_zone) {
        throw std::runtime_error("ZoneSelector: goal position is not inside any zone");
    }

    ZoneSelectionResult result;
    result.selected_zones.push_back(start_zone);
    if (start_zone != goal_zone) {
        result.selected_zones.push_back(goal_zone);
    }

    const auto resolvedZoneBehavior =
        [&](const std::shared_ptr<zones::Zone>& zone,
            const std::string& fallback_behavior) {
            if (zone == nullptr) {
                return fallback_behavior;
            }
            const std::string resolved = zone->getResolvedPlannerBehavior();
            return resolved.empty() ? fallback_behavior : resolved;
        };

    result.frontiers.push_back(
        SearchFrontierDescriptor{0, SearchFrontierRole::StartZone, start_zone, start_frontier_behavior});

    if (start_zone == goal_zone) {
        result.search_boundary = correctedPolygon(start_zone->getPolygon());
        return result;
    }

    if (!arePolygonsDirectlyConnected(start_zone->getPolygon(), goal_zone->getPolygon())) {
        throw std::runtime_error(
            "ZoneSelector: start and goal zones do not share an edge or overlap");
    }

    result.frontiers.push_back(SearchFrontierDescriptor{
        1,
        SearchFrontierRole::GoalZone,
        goal_zone,
        resolvedZoneBehavior(goal_zone, start_frontier_behavior)
    });
    result.search_boundary = exactUnionBoundary(start_zone->getPolygon(), goal_zone->getPolygon());

    return result;
}

ZoneConnectivityIndex ZoneSelector::buildConnectivityIndex(
    const std::vector<std::shared_ptr<zones::Zone>>& zones) {
    ZoneConnectivityIndex connectivity;
    connectivity.adjacent_zone_indices.resize(zones.size());

    for (size_t first = 0; first < zones.size(); ++first) {
        for (size_t second = first + 1; second < zones.size(); ++second) {
            if (!zones[first] || !zones[second]) {
                continue;
            }
            if (!arePolygonsDirectlyConnected(zones[first]->getPolygon(), zones[second]->getPolygon())) {
                continue;
            }
            connectivity.adjacent_zone_indices[first].push_back(second);
            connectivity.adjacent_zone_indices[second].push_back(first);
        }
    }

    return connectivity;
}

std::shared_ptr<zones::Zone> ZoneSelector::findContainingZone(
    const geometry::Point2d& point,
    const std::vector<std::shared_ptr<zones::Zone>>& zones) {
    for (const auto& zone : zones) {
        if (isInsidePolygon(point, zone->getPolygon())) {
            return zone;
        }
    }
    return nullptr;
}

bool ZoneSelector::isInsidePolygon(
    const geometry::Point2d& point,
    const geometry::Polygon2d& polygon) {
    return geometry::bg::within(point, polygon);
}

} // namespace costs
} // namespace coastmotionplanning
