#include "coastmotionplanning/costs/zone_selector.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/correct.hpp>

namespace coastmotionplanning {
namespace costs {

namespace {

// ---- Alpha-Shape / Concave Hull Helpers ----

struct Triangle {
    size_t a, b, c;  // indices into point array
};

// Compute circumradius of a triangle given 3 points
double circumradius(const geometry::Point2d& p1,
                    const geometry::Point2d& p2,
                    const geometry::Point2d& p3) {
    double ax = geometry::bg::get<0>(p1);
    double ay = geometry::bg::get<1>(p1);
    double bx = geometry::bg::get<0>(p2);
    double by = geometry::bg::get<1>(p2);
    double cx = geometry::bg::get<0>(p3);
    double cy = geometry::bg::get<1>(p3);

    double a_len = std::sqrt((bx - cx) * (bx - cx) + (by - cy) * (by - cy));
    double b_len = std::sqrt((ax - cx) * (ax - cx) + (ay - cy) * (ay - cy));
    double c_len = std::sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));

    double s = (a_len + b_len + c_len) / 2.0;
    double area = std::sqrt(std::max(0.0, s * (s - a_len) * (s - b_len) * (s - c_len)));

    if (area < 1e-12) return std::numeric_limits<double>::max();

    return (a_len * b_len * c_len) / (4.0 * area);
}

// Simple Delaunay-like triangulation using ear clipping on the point set.
// For robustness, we use a convex hull + fan approach for the alpha shape.
// This is a simplified version: we compute the Delaunay triangulation via
// a sweep-line incremental approach, then filter by alpha.
//
// For production robustness, we use a simpler but effective approach:
// 1. Compute the convex hull of all points
// 2. For every possible triangle formed by the hull + interior points,
//    filter by circumradius <= 1/alpha
// 3. Extract boundary edges
//
// Since our point sets are small (zone vertices, typically < 100 points),
// an O(n^3) approach is acceptable.

struct EdgeKey {
    size_t a, b;
    EdgeKey(size_t x, size_t y) : a(std::min(x, y)), b(std::max(x, y)) {}
    bool operator==(const EdgeKey& o) const { return a == o.a && b == o.b; }
};

struct EdgeKeyHash {
    size_t operator()(const EdgeKey& e) const {
        return std::hash<size_t>()(e.a) ^ (std::hash<size_t>()(e.b) << 32);
    }
};

// Cross product of vectors (p1->p2) x (p1->p3)
double cross2d(const geometry::Point2d& p1,
               const geometry::Point2d& p2,
               const geometry::Point2d& p3) {
    return (geometry::bg::get<0>(p2) - geometry::bg::get<0>(p1)) *
           (geometry::bg::get<1>(p3) - geometry::bg::get<1>(p1)) -
           (geometry::bg::get<1>(p2) - geometry::bg::get<1>(p1)) *
           (geometry::bg::get<0>(p3) - geometry::bg::get<0>(p1));
}

// Check if point d is inside the circumcircle of triangle (a, b, c)
// Assumes a, b, c are in counter-clockwise order
bool inCircumcircle(const geometry::Point2d& a,
                    const geometry::Point2d& b,
                    const geometry::Point2d& c,
                    const geometry::Point2d& d) {
    double ax = geometry::bg::get<0>(a) - geometry::bg::get<0>(d);
    double ay = geometry::bg::get<1>(a) - geometry::bg::get<1>(d);
    double bx = geometry::bg::get<0>(b) - geometry::bg::get<0>(d);
    double by = geometry::bg::get<1>(b) - geometry::bg::get<1>(d);
    double cx = geometry::bg::get<0>(c) - geometry::bg::get<0>(d);
    double cy = geometry::bg::get<1>(c) - geometry::bg::get<1>(d);

    double det = ax * (by * (cx * cx + cy * cy) - cy * (bx * bx + by * by))
               - ay * (bx * (cx * cx + cy * cy) - cx * (bx * bx + by * by))
               + (ax * ax + ay * ay) * (bx * cy - by * cx);
    return det > 0;
}

// Build a simple Delaunay triangulation using the Bowyer-Watson algorithm
std::vector<Triangle> delaunayTriangulation(const std::vector<geometry::Point2d>& points) {
    if (points.size() < 3) return {};

    // Find bounding box
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    for (const auto& p : points) {
        min_x = std::min(min_x, geometry::bg::get<0>(p));
        min_y = std::min(min_y, geometry::bg::get<1>(p));
        max_x = std::max(max_x, geometry::bg::get<0>(p));
        max_y = std::max(max_y, geometry::bg::get<1>(p));
    }

    double dx = max_x - min_x;
    double dy = max_y - min_y;
    double delta_max = std::max(dx, dy);
    double mid_x = (min_x + max_x) / 2.0;
    double mid_y = (min_y + max_y) / 2.0;

    // Create super-triangle that encompasses all points
    // We'll add 3 super-triangle vertices at the end of the points array
    size_t n = points.size();
    std::vector<geometry::Point2d> all_pts = points;
    all_pts.emplace_back(mid_x - 20.0 * delta_max, mid_y - delta_max);
    all_pts.emplace_back(mid_x, mid_y + 20.0 * delta_max);
    all_pts.emplace_back(mid_x + 20.0 * delta_max, mid_y - delta_max);

    std::vector<Triangle> triangles;
    triangles.push_back({n, n + 1, n + 2});

    // Insert each point
    for (size_t i = 0; i < n; ++i) {
        std::vector<Triangle> bad_triangles;
        for (const auto& tri : triangles) {
            // Ensure CCW ordering for circumcircle test
            double c = cross2d(all_pts[tri.a], all_pts[tri.b], all_pts[tri.c]);
            size_t a_idx = tri.a, b_idx = tri.b, c_idx = tri.c;
            if (c < 0) std::swap(b_idx, c_idx);
            if (inCircumcircle(all_pts[a_idx], all_pts[b_idx], all_pts[c_idx], all_pts[i])) {
                bad_triangles.push_back(tri);
            }
        }

        // Find the boundary polygon of the bad triangles (polygon hole)
        std::unordered_map<size_t, std::unordered_map<size_t, int>> edge_count;
        for (const auto& tri : bad_triangles) {
            size_t edges[3][2] = {{tri.a, tri.b}, {tri.b, tri.c}, {tri.c, tri.a}};
            for (auto& e : edges) {
                size_t ea = std::min(e[0], e[1]);
                size_t eb = std::max(e[0], e[1]);
                edge_count[ea][eb]++;
            }
        }

        // Remove bad triangles
        triangles.erase(
            std::remove_if(triangles.begin(), triangles.end(),
                [&bad_triangles](const Triangle& t) {
                    for (const auto& bt : bad_triangles) {
                        if (t.a == bt.a && t.b == bt.b && t.c == bt.c) return true;
                    }
                    return false;
                }),
            triangles.end());

        // Add new triangles from boundary edges to the new point
        for (const auto& [ea, inner] : edge_count) {
            for (const auto& [eb, count] : inner) {
                if (count == 1) {
                    triangles.push_back({ea, eb, i});
                }
            }
        }
    }

    // Remove triangles that share vertices with super-triangle
    triangles.erase(
        std::remove_if(triangles.begin(), triangles.end(),
            [n](const Triangle& t) {
                return t.a >= n || t.b >= n || t.c >= n;
            }),
        triangles.end());

    return triangles;
}

// Extract the boundary polygon from a set of triangles (edges that appear exactly once)
geometry::Polygon2d extractBoundary(const std::vector<Triangle>& triangles,
                                     const std::vector<geometry::Point2d>& points) {
    // Count edge occurrences
    std::unordered_map<size_t, std::unordered_map<size_t, int>> edge_count;
    // Also store directed edges for ordering
    std::unordered_map<size_t, std::vector<size_t>> adjacency;

    for (const auto& tri : triangles) {
        size_t verts[3] = {tri.a, tri.b, tri.c};
        for (int i = 0; i < 3; ++i) {
            size_t ea = std::min(verts[i], verts[(i + 1) % 3]);
            size_t eb = std::max(verts[i], verts[(i + 1) % 3]);
            edge_count[ea][eb]++;
        }
    }

    // Boundary edges are those that appear exactly once
    for (const auto& [ea, inner] : edge_count) {
        for (const auto& [eb, count] : inner) {
            if (count == 1) {
                adjacency[ea].push_back(eb);
                adjacency[eb].push_back(ea);
            }
        }
    }

    if (adjacency.empty()) {
        return geometry::Polygon2d();
    }

    // Walk the boundary
    std::vector<size_t> boundary_idx;
    std::unordered_set<size_t> visited_edges_set;
    size_t start = adjacency.begin()->first;
    size_t current = start;
    size_t prev = std::numeric_limits<size_t>::max();

    do {
        boundary_idx.push_back(current);
        bool found_next = false;
        for (size_t next : adjacency[current]) {
            if (next == prev) continue;
            EdgeKey ek(current, next);
            size_t edge_hash = EdgeKeyHash{}(ek);
            if (visited_edges_set.count(edge_hash)) continue;
            visited_edges_set.insert(edge_hash);
            prev = current;
            current = next;
            found_next = true;
            break;
        }
        if (!found_next) break;
    } while (current != start && boundary_idx.size() < points.size() + 1);

    geometry::Polygon2d result;
    for (size_t idx : boundary_idx) {
        result.outer().push_back(points[idx]);
    }
    // Close the polygon
    if (!result.outer().empty()) {
        result.outer().push_back(result.outer().front());
    }
    geometry::bg::correct(result);
    return result;
}

} // anonymous namespace

// =============================================================================
// ZoneSelector implementation
// =============================================================================

ZoneSelectionResult ZoneSelector::select(
    const math::Pose2d& start,
    const math::Pose2d& goal,
    const std::vector<std::shared_ptr<zones::Zone>>& all_zones,
    double alpha) const {

    geometry::Point2d start_pt(start.x, start.y);
    geometry::Point2d goal_pt(goal.x, goal.y);

    auto start_zone = findContainingZone(start_pt, all_zones);
    auto goal_zone = findContainingZone(goal_pt, all_zones);

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

    // Collect zone polygons
    std::vector<geometry::Polygon2d> zone_polygons;
    for (const auto& z : result.selected_zones) {
        zone_polygons.push_back(z->getPolygon());
    }

    // Compute concave hull search boundary
    result.search_boundary = computeConcaveHull(zone_polygons, alpha);

    // Verify that start and goal are inside the boundary.
    // If the alpha-shape is too tight, fall back to convex hull.
    if (!isInsidePolygon(start_pt, result.search_boundary) ||
        !isInsidePolygon(goal_pt, result.search_boundary)) {
        // Fallback: use convex hull which is guaranteed to contain all zone vertices
        geometry::Polygon2d pts_poly;
        for (const auto& poly : zone_polygons) {
            for (const auto& pt : poly.outer()) {
                pts_poly.outer().push_back(pt);
            }
        }
        if (!pts_poly.outer().empty()) {
            pts_poly.outer().push_back(pts_poly.outer().front());
        }
        geometry::bg::convex_hull(pts_poly, result.search_boundary);
    }

    return result;
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

geometry::Polygon2d ZoneSelector::computeConcaveHull(
    const std::vector<geometry::Polygon2d>& zone_polygons,
    double alpha) {
    // Collect all unique vertices from all zone polygons
    std::vector<geometry::Point2d> all_points;
    for (const auto& poly : zone_polygons) {
        for (const auto& pt : poly.outer()) {
            all_points.push_back(pt);
        }
    }

    // Remove duplicate closing vertices
    if (all_points.size() > 1) {
        auto it = std::unique(all_points.begin(), all_points.end(),
            [](const geometry::Point2d& a, const geometry::Point2d& b) {
                return std::abs(geometry::bg::get<0>(a) - geometry::bg::get<0>(b)) < 1e-9 &&
                       std::abs(geometry::bg::get<1>(a) - geometry::bg::get<1>(b)) < 1e-9;
            });
        all_points.erase(it, all_points.end());
    }

    if (all_points.size() < 3) {
        // Not enough points for a polygon, return convex hull
        geometry::Polygon2d hull;
        geometry::Polygon2d pts_poly;
        for (const auto& pt : all_points) {
            pts_poly.outer().push_back(pt);
        }
        geometry::bg::convex_hull(pts_poly, hull);
        return hull;
    }

    // If only one zone polygon, just return it (with corrections)
    if (zone_polygons.size() == 1) {
        geometry::Polygon2d result = zone_polygons[0];
        geometry::bg::correct(result);
        return result;
    }

    // Auto-derive alpha if not specified
    if (alpha <= 0.0) {
        // Compute bounding box diagonal, use a fraction of it
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();
        for (const auto& pt : all_points) {
            min_x = std::min(min_x, geometry::bg::get<0>(pt));
            min_y = std::min(min_y, geometry::bg::get<1>(pt));
            max_x = std::max(max_x, geometry::bg::get<0>(pt));
            max_y = std::max(max_y, geometry::bg::get<1>(pt));
        }
        double diag = std::sqrt((max_x - min_x) * (max_x - min_x) +
                                (max_y - min_y) * (max_y - min_y));
        // alpha = 1 / (max allowable circumradius)
        // We allow circumradius up to ~1/4 of the diagonal to bridge gaps
        alpha = 1.0 / (diag * 0.25);
    }

    // Compute Delaunay triangulation
    auto triangles = delaunayTriangulation(all_points);

    // Filter triangles by alpha criterion: keep if circumradius <= 1/alpha
    double max_circumradius = 1.0 / alpha;
    std::vector<Triangle> alpha_triangles;
    for (const auto& tri : triangles) {
        double cr = circumradius(all_points[tri.a], all_points[tri.b], all_points[tri.c]);
        if (cr <= max_circumradius) {
            alpha_triangles.push_back(tri);
        }
    }

    if (alpha_triangles.empty()) {
        // Fallback to convex hull if alpha is too aggressive
        geometry::Polygon2d hull;
        geometry::Polygon2d pts_poly;
        for (const auto& pt : all_points) {
            pts_poly.outer().push_back(pt);
        }
        if (!pts_poly.outer().empty()) {
            pts_poly.outer().push_back(pts_poly.outer().front());
        }
        geometry::bg::convex_hull(pts_poly, hull);
        return hull;
    }

    // Extract boundary
    return extractBoundary(alpha_triangles, all_points);
}

bool ZoneSelector::isInsidePolygon(
    const geometry::Point2d& point,
    const geometry::Polygon2d& polygon) {
    return geometry::bg::within(point, polygon);
}

} // namespace costs
} // namespace coastmotionplanning
