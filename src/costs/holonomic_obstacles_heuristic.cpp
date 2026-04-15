#include "coastmotionplanning/costs/holonomic_obstacles_heuristic.hpp"

#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <vector>

namespace coastmotionplanning {
namespace costs {

namespace {

HolonomicHeuristicLayerStats buildHeuristicLayer(
    grid_map::GridMap& costmap,
    const std::string& layer_name,
    const std::vector<grid_map::Index>& seed_indices) {

    const double resolution = costmap.getResolution();
    const auto& size = costmap.getSize();
    const int rows = size(0);
    const int cols = size(1);

    const float nan_value = std::numeric_limits<float>::quiet_NaN();
    if (costmap.exists(layer_name)) {
        costmap[layer_name].setConstant(nan_value);
    } else {
        costmap.add(layer_name, nan_value);
    }

    auto& obstacle_data = costmap[CostmapLayerNames::STATIC_OBSTACLES];
    auto& heuristic_data = costmap[layer_name];
    HolonomicHeuristicLayerStats stats;

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

    for (const auto& seed_idx : seed_indices) {
        const int seed_r = seed_idx(0);
        const int seed_c = seed_idx(1);
        if (seed_r < 0 || seed_r >= rows || seed_c < 0 || seed_c >= cols) {
            continue;
        }
        if (obstacle_data(seed_r, seed_c) >= CostValues::LETHAL) {
            continue;
        }
        if (dist_grid(seed_r, seed_c) <= 0.0f) {
            continue;
        }
        dist_grid(seed_r, seed_c) = 0.0f;
        pq.push({0.0f, seed_r, seed_c});
        ++stats.seed_cell_count;
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
                ++stats.finite_cell_count;
                const double distance = static_cast<double>(d);
                if (stats.finite_cell_count == 1) {
                    stats.min_finite_value = distance;
                    stats.max_finite_value = distance;
                } else {
                    stats.min_finite_value = std::min(stats.min_finite_value, distance);
                    stats.max_finite_value = std::max(stats.max_finite_value, distance);
                }
            }
            // Cells still at max remain NaN (unreachable)
        }
    }

    return stats;
}

} // namespace

HolonomicHeuristicLayerStats HolonomicObstaclesHeuristic::compute(
    grid_map::GridMap& costmap,
    const grid_map::Position& goal_position) {
    grid_map::Index goal_idx;
    if (!costmap.getIndex(goal_position, goal_idx)) {
        if (costmap.exists(CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES)) {
            costmap[CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES].setConstant(
                std::numeric_limits<float>::quiet_NaN());
        } else {
            costmap.add(
                CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES,
                std::numeric_limits<float>::quiet_NaN());
        }
        return {};
    }

    return computeFromSeedCells(
        costmap,
        CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES,
        std::vector<grid_map::Index>{goal_idx});
}

HolonomicHeuristicLayerStats HolonomicObstaclesHeuristic::computeFromSeedCells(
    grid_map::GridMap& costmap,
    const std::string& layer_name,
    const std::vector<grid_map::Index>& seed_indices) {
    return buildHeuristicLayer(costmap, layer_name, seed_indices);
}

std::string HolonomicObstaclesHeuristic::makeStageLayerName(
    size_t source_frontier_id,
    size_t target_frontier_id) {
    return "stage_holonomic_frontier_" + std::to_string(source_frontier_id) +
        "_to_" + std::to_string(target_frontier_id);
}

} // namespace costs
} // namespace coastmotionplanning
