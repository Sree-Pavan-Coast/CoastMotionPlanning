#include "coastmotionplanning/costs/zone_selector.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
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

// Extract boundary edges from a set of triangles (edges that appear exactly once).
// Returns one or more closed rings. When multiple disconnected boundary loops exist
// (e.g. two separate zone polygons connected only by thin alpha triangles), each
// loop is extracted independently and the largest loop by area is returned.
// At junction vertices the walk picks the neighbor that makes the tightest
// counter-clockwise turn, which avoids self-intersecting paths.
geometry::Polygon2d extractBoundary(const std::vector<Triangle>& triangles,
                                     const std::vector<geometry::Point2d>& points) {
    // --- 1. Collect boundary edges (appear in exactly one triangle) ---
    std::map<std::pair<size_t, size_t>, int> edge_count;
    for (const auto& tri : triangles) {
        size_t verts[3] = {tri.a, tri.b, tri.c};
        for (int i = 0; i < 3; ++i) {
            auto ea = std::min(verts[i], verts[(i + 1) % 3]);
            auto eb = std::max(verts[i], verts[(i + 1) % 3]);
            edge_count[{ea, eb}]++;
        }
    }

    std::map<size_t, std::set<size_t>> adjacency;
    for (const auto& [edge, count] : edge_count) {
        if (count == 1) {
            adjacency[edge.first].insert(edge.second);
            adjacency[edge.second].insert(edge.first);
        }
    }

    if (adjacency.empty()) {
        return geometry::Polygon2d();
    }

    // --- 2. Helper: angle from point `from` to point `to` ---
    auto angleTo = [&](size_t from, size_t to) -> double {
        double dx = geometry::bg::get<0>(points[to]) - geometry::bg::get<0>(points[from]);
        double dy = geometry::bg::get<1>(points[to]) - geometry::bg::get<1>(points[from]);
        return std::atan2(dy, dx);
    };

    // --- 3. Walk boundary loops using angle-ordered traversal ---
    // At each vertex pick the neighbor that makes the smallest CCW turn from
    // the incoming direction. This keeps the walk on the outer boundary and
    // avoids crossing into inner loops.
    std::set<std::pair<size_t, size_t>> used_edges;
    std::vector<std::vector<size_t>> loops;

    // Find starting vertex: pick the vertex with the smallest x (then y) —
    // guaranteed to be on an outer boundary.
    auto pick_start = [&]() -> size_t {
        size_t best = adjacency.begin()->first;
        for (const auto& [v, _] : adjacency) {
            // Skip vertices whose edges are all consumed
            bool has_free = false;
            for (size_t nb : adjacency[v]) {
                auto e = std::make_pair(std::min(v, nb), std::max(v, nb));
                if (!used_edges.count(e)) {
                    has_free = true;
                    break;
                }
            }
            if (!has_free) continue;

            bool has_free_best = false;
            for (size_t nb : adjacency[best]) {
                auto e = std::make_pair(std::min(best, nb), std::max(best, nb));
                if (!used_edges.count(e)) {
                    has_free_best = true;
                    break;
                }
            }
            if (!has_free_best) {
                best = v;
                continue;
            }

            double vx = geometry::bg::get<0>(points[v]);
            double vy = geometry::bg::get<1>(points[v]);
            double bx = geometry::bg::get<0>(points[best]);
            double by = geometry::bg::get<1>(points[best]);
            if (vx < bx || (vx == bx && vy < by)) {
                best = v;
            }
        }
        return best;
    };

    // Extract all boundary loops
    size_t total_boundary_edges = 0;
    for (const auto& [edge, count] : edge_count) {
        if (count == 1) total_boundary_edges++;
    }

    while (used_edges.size() < total_boundary_edges) {
        size_t start = pick_start();

        // Pick the initial direction: from start, go to the neighbor with the
        // largest angle (most CCW from +x). For the leftmost point this means
        // we begin walking the outer boundary counter-clockwise.
        size_t first_next = *adjacency[start].begin();
        double best_angle = -4.0; // < -pi
        for (size_t nb : adjacency[start]) {
            auto e = std::make_pair(std::min(start, nb), std::max(start, nb));
            if (used_edges.count(e)) continue;
            double a = angleTo(start, nb);
            if (a > best_angle) {
                best_angle = a;
                first_next = nb;
            }
        }

        std::vector<size_t> loop;
        loop.push_back(start);

        size_t prev = start;
        size_t current = first_next;
        {
            auto e = std::make_pair(std::min(prev, current), std::max(prev, current));
            used_edges.insert(e);
        }

        while (current != start && loop.size() < points.size() + 2) {
            loop.push_back(current);

            // Incoming angle (from prev to current)
            double incoming = angleTo(prev, current);

            // Among unused neighbors of current, pick the one that makes the
            // smallest left (CCW) turn from the incoming direction.
            // Turn angle = neighbor_angle - incoming, normalized to (-pi, pi].
            // We want the most negative turn (tightest right / least left) to
            // stay on the outer boundary.
            size_t best_next = current;
            double best_turn = 10.0; // > 2*pi, will be replaced
            for (size_t nb : adjacency[current]) {
                if (nb == prev) {
                    auto e = std::make_pair(std::min(current, nb), std::max(current, nb));
                    if (used_edges.count(e)) continue;
                }
                auto e = std::make_pair(std::min(current, nb), std::max(current, nb));
                if (used_edges.count(e)) continue;

                double outgoing = angleTo(current, nb);
                double turn = outgoing - incoming;
                // Normalize to (-pi, pi]
                while (turn > M_PI) turn -= 2.0 * M_PI;
                while (turn <= -M_PI) turn += 2.0 * M_PI;

                if (turn < best_turn) {
                    best_turn = turn;
                    best_next = nb;
                }
            }

            if (best_next == current) break; // no unvisited neighbor

            auto e = std::make_pair(std::min(current, best_next), std::max(current, best_next));
            used_edges.insert(e);
            prev = current;
            current = best_next;
        }

        if (loop.size() >= 3) {
            loops.push_back(std::move(loop));
        }
    }

    if (loops.empty()) {
        return geometry::Polygon2d();
    }

    // --- 4. Pick the loop with the largest absolute area ---
    size_t best_loop = 0;
    double best_area = 0.0;
    for (size_t i = 0; i < loops.size(); ++i) {
        geometry::Polygon2d poly;
        for (size_t idx : loops[i]) {
            poly.outer().push_back(points[idx]);
        }
        if (!poly.outer().empty()) {
            poly.outer().push_back(poly.outer().front());
        }
        geometry::bg::correct(poly);
        double a = std::abs(geometry::bg::area(poly));
        if (a > best_area) {
            best_area = a;
            best_loop = i;
        }
    }

    geometry::Polygon2d result;
    for (size_t idx : loops[best_loop]) {
        result.outer().push_back(points[idx]);
    }
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
    double alpha,
    const std::string& start_frontier_behavior,
    const std::string& transition_frontier_behavior) const {

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

    const auto resolvedZoneBehavior =
        [&](const std::shared_ptr<zones::Zone>& zone,
            const std::string& fallback_behavior) {
            if (zone == nullptr) {
                return fallback_behavior;
            }
            const std::string resolved = zone->getResolvedPlannerBehavior();
            return resolved.empty() ? fallback_behavior : resolved;
        };

    result.frontiers.push_back(SearchFrontierDescriptor{
        0,
        SearchFrontierRole::StartZone,
        start_zone,
        start_frontier_behavior,
        std::nullopt
    });
    if (start_zone != goal_zone) {
        result.frontiers.push_back(SearchFrontierDescriptor{
            1,
            SearchFrontierRole::Transition,
            nullptr,
            transition_frontier_behavior.empty()
                ? resolvedZoneBehavior(start_zone, start_frontier_behavior)
                : transition_frontier_behavior,
            2
        });
        result.frontiers.push_back(SearchFrontierDescriptor{
            2,
            SearchFrontierRole::GoalZone,
            goal_zone,
            resolvedZoneBehavior(goal_zone, start_frontier_behavior),
            std::nullopt
        });
        result.frontiers[0].next_frontier_id = 1;
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
            const bool already_present = std::any_of(
                all_points.begin(),
                all_points.end(),
                [&](const geometry::Point2d& existing) {
                    return geometry::arePointsClose(existing, pt, 1e-9);
                });
            if (!already_present) {
                all_points.push_back(pt);
            }
        }
    }

    auto makeConvexHull = [&]() {
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
    };

    if (all_points.size() < 3) {
        return makeConvexHull();
    }

    // If only one zone polygon, just return it (with corrections)
    if (zone_polygons.size() == 1) {
        geometry::Polygon2d result = zone_polygons[0];
        geometry::bg::correct(result);
        return result;
    }

    // Compute Delaunay triangulation
    auto triangles = delaunayTriangulation(all_points);
    if (triangles.empty()) {
        return makeConvexHull();
    }

    // Sort all triangle circumradii so we can binary-search for the minimum
    // threshold that produces a connected alpha shape.
    std::vector<double> tri_radii(triangles.size());
    for (size_t i = 0; i < triangles.size(); ++i) {
        tri_radii[i] = circumradius(
            all_points[triangles[i].a],
            all_points[triangles[i].b],
            all_points[triangles[i].c]);
    }

    // Check if a set of triangles forms a single connected component using
    // Union-Find over vertex indices.
    auto coversAllPointsAndIsConnected =
        [&](const std::vector<size_t>& tri_indices, size_t num_points) -> bool {
        std::vector<size_t> parent(num_points);
        std::iota(parent.begin(), parent.end(), 0);
        std::function<size_t(size_t)> find = [&](size_t x) -> size_t {
            return parent[x] == x ? x : (parent[x] = find(parent[x]));
        };
        auto unite = [&](size_t a, size_t b) {
            a = find(a);
            b = find(b);
            if (a != b) parent[a] = b;
        };

        // Collect which vertices appear in the triangle set
        std::unordered_set<size_t> active_verts;
        for (size_t idx : tri_indices) {
            const auto& tri = triangles[idx];
            active_verts.insert(tri.a);
            active_verts.insert(tri.b);
            active_verts.insert(tri.c);
            unite(tri.a, tri.b);
            unite(tri.b, tri.c);
        }

        if (active_verts.size() != num_points) return false;
        size_t root = find(*active_verts.begin());
        for (size_t v : active_verts) {
            if (find(v) != root) return false;
        }
        return true;
    };

    // Gather sorted unique circumradii thresholds
    std::vector<double> sorted_radii = tri_radii;
    std::sort(sorted_radii.begin(), sorted_radii.end());
    sorted_radii.erase(
        std::unique(sorted_radii.begin(), sorted_radii.end()),
        sorted_radii.end());

    // If a user-specified alpha is provided, try it first
    double max_circumradius = 0.0;
    if (alpha > 0.0) {
        max_circumradius = 1.0 / alpha;
    }

    // Filter triangles by the chosen threshold and check connectivity.
    // If disconnected (or no alpha specified), find the minimum threshold
    // from the sorted radii that produces a connected set.
    auto filterTriangles = [&](double threshold) -> std::vector<size_t> {
        std::vector<size_t> indices;
        for (size_t i = 0; i < triangles.size(); ++i) {
            if (tri_radii[i] <= threshold) {
                indices.push_back(i);
            }
        }
        return indices;
    };

    bool found_connected = false;
    std::vector<size_t> selected_indices;

    if (alpha > 0.0) {
        selected_indices = filterTriangles(max_circumradius);
        if (!selected_indices.empty() &&
            coversAllPointsAndIsConnected(selected_indices, all_points.size())) {
            found_connected = true;
        }
    }

    if (!found_connected) {
        // Walk sorted radii from small to large until we get a connected set
        for (double threshold : sorted_radii) {
            selected_indices = filterTriangles(threshold);
            if (!selected_indices.empty() &&
                coversAllPointsAndIsConnected(selected_indices, all_points.size())) {
                max_circumradius = threshold;
                found_connected = true;
                break;
            }
        }
    }

    if (!found_connected || selected_indices.empty()) {
        return makeConvexHull();
    }

    // Build the final triangle list
    std::vector<Triangle> alpha_triangles;
    alpha_triangles.reserve(selected_indices.size());
    for (size_t idx : selected_indices) {
        alpha_triangles.push_back(triangles[idx]);
    }

    auto boundary = extractBoundary(alpha_triangles, all_points);

    // Sanity check: the boundary must be a valid non-degenerate polygon
    if (boundary.outer().size() < 4) { // 3 vertices + closing
        return makeConvexHull();
    }

    for (const auto& pt : all_points) {
        if (!geometry::bg::covered_by(pt, boundary)) {
            return makeConvexHull();
        }
    }

    return boundary;
}

bool ZoneSelector::isInsidePolygon(
    const geometry::Point2d& point,
    const geometry::Polygon2d& polygon) {
    return geometry::bg::within(point, polygon);
}

} // namespace costs
} // namespace coastmotionplanning
