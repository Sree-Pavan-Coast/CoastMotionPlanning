#include "coastmotionplanning/costs/holonomic_obstacles_heuristic.hpp"

#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace coastmotionplanning {
namespace costs {

void HolonomicObstaclesHeuristic::compute(
    grid_map::GridMap& costmap,
    const grid_map::Position& goal_position) {

    const double resolution = costmap.getResolution();
    const auto& size = costmap.getSize();
    const int rows = size(0);
    const int cols = size(1);

    // Add the heuristic layer, initialized to NaN (unreachable by default)
    costmap.add(CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES,
                std::numeric_limits<float>::quiet_NaN());

    auto& obstacle_data = costmap[CostmapLayerNames::STATIC_OBSTACLES];
    auto& heuristic_data = costmap[CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES];

    // Find the goal cell index
    grid_map::Index goal_idx;
    if (!costmap.getIndex(goal_position, goal_idx)) {
        return;  // Goal is outside the map
    }

    // Distance grid
    Eigen::MatrixXf dist_grid(rows, cols);
    dist_grid.setConstant(std::numeric_limits<float>::max());

    // Priority queue: (distance, row, col)
    struct Cell {
        float distance;
        int row, col;
        bool operator>(const Cell& other) const { return distance > other.distance; }
    };
    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> pq;

    // Seed from goal
    int goal_r = goal_idx(0);
    int goal_c = goal_idx(1);
    dist_grid(goal_r, goal_c) = 0.0f;
    pq.push({0.0f, goal_r, goal_c});

    // 8-connected neighbors
    const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const float cardinal = static_cast<float>(resolution);
    const float diagonal = static_cast<float>(resolution * std::sqrt(2.0));
    const float dd[] = {diagonal, cardinal, diagonal, cardinal,
                        cardinal, diagonal, cardinal, diagonal};

    while (!pq.empty()) {
        Cell cur = pq.top();
        pq.pop();

        if (cur.distance > dist_grid(cur.row, cur.col)) {
            continue;  // Stale entry
        }

        for (int i = 0; i < 8; ++i) {
            int nr = cur.row + dx[i];
            int nc = cur.col + dy[i];

            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;

            // Skip lethal cells (obstacles are impassable)
            if (obstacle_data(nr, nc) >= CostValues::LETHAL) continue;

            float new_dist = cur.distance + dd[i];

            if (new_dist < dist_grid(nr, nc)) {
                dist_grid(nr, nc) = new_dist;
                pq.push({new_dist, nr, nc});
            }
        }
    }

    // Copy distances to the heuristic layer
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            float d = dist_grid(r, c);
            if (d < std::numeric_limits<float>::max()) {
                heuristic_data(r, c) = d;
            }
            // Cells still at max remain NaN (unreachable)
        }
    }
}

} // namespace costs
} // namespace coastmotionplanning
