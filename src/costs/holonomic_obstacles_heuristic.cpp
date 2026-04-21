#include "coastmotionplanning/costs/holonomic_obstacles_heuristic.hpp"

#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace coastmotionplanning {
namespace costs {

grid_map::Matrix HolonomicObstaclesHeuristic::computeField(
    const grid_map::GridMap& costmap,
    const std::vector<grid_map::Position>& seed_positions) {
    const double resolution = costmap.getResolution();
    const auto& size = costmap.getSize();
    const int rows = size(0);
    const int cols = size(1);

    grid_map::Matrix heuristic_data(rows, cols);
    heuristic_data.setConstant(std::numeric_limits<float>::quiet_NaN());
    if (!costmap.exists(CostmapLayerNames::STATIC_OBSTACLES)) {
        return heuristic_data;
    }

    const auto& obstacle_data = costmap[CostmapLayerNames::STATIC_OBSTACLES];

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

    bool seeded = false;
    for (const auto& seed_position : seed_positions) {
        grid_map::Index seed_idx;
        if (!costmap.getIndex(seed_position, seed_idx)) {
            continue;
        }

        const int seed_r = seed_idx(0);
        const int seed_c = seed_idx(1);
        if (obstacle_data(seed_r, seed_c) >= CostValues::LETHAL) {
            continue;
        }

        if (dist_grid(seed_r, seed_c) <= 0.0f) {
            continue;
        }
        dist_grid(seed_r, seed_c) = 0.0f;
        pq.push({0.0f, seed_r, seed_c});
        seeded = true;
    }
    if (!seeded) {
        return heuristic_data;
    }

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

    return heuristic_data;
}

void HolonomicObstaclesHeuristic::compute(
    grid_map::GridMap& costmap,
    const grid_map::Position& goal_position) {
    compute(costmap, std::vector<grid_map::Position>{goal_position});
}

void HolonomicObstaclesHeuristic::compute(
    grid_map::GridMap& costmap,
    const std::vector<grid_map::Position>& seed_positions) {
    costmap.add(CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES,
                std::numeric_limits<float>::quiet_NaN());
    costmap[CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES] =
        computeField(costmap, seed_positions);
}

} // namespace costs
} // namespace coastmotionplanning
