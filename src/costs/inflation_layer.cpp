#include "coastmotionplanning/costs/inflation_layer.hpp"

#include <cmath>
#include <queue>
#include <vector>

namespace coastmotionplanning {
namespace costs {

void InflationLayer::build(grid_map::GridMap& costmap,
                            double inflation_radius_m,
                            double inscribed_radius_m,
                            double cost_scaling_factor) {
    const double resolution = costmap.getResolution();
    const auto& size = costmap.getSize();
    const int rows = size(0);
    const int cols = size(1);

    // Add the inflation layer, initialized to 0 (free)
    costmap.add(CostmapLayerNames::INFLATION, CostValues::FREE_SPACE);

    auto& obstacle_data = costmap[CostmapLayerNames::STATIC_OBSTACLES];
    auto& inflation_data = costmap[CostmapLayerNames::INFLATION];

    // Distance grid (in meters), initialized to infinity
    Eigen::MatrixXf distance_grid(rows, cols);
    distance_grid.setConstant(std::numeric_limits<float>::max());

    // BFS queue: (row, col)
    struct Cell {
        int row, col;
        float distance;
        bool operator>(const Cell& other) const { return distance > other.distance; }
    };
    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> queue;

    // Seed the queue with all lethal cells (distance = 0)
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (obstacle_data(r, c) >= CostValues::LETHAL) {
                distance_grid(r, c) = 0.0f;
                queue.push({r, c, 0.0f});
            }
        }
    }

    // 8-connected neighbor offsets with distances
    const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const float dd[] = {
        static_cast<float>(resolution * std::sqrt(2.0)),
        static_cast<float>(resolution),
        static_cast<float>(resolution * std::sqrt(2.0)),
        static_cast<float>(resolution),
        static_cast<float>(resolution),
        static_cast<float>(resolution * std::sqrt(2.0)),
        static_cast<float>(resolution),
        static_cast<float>(resolution * std::sqrt(2.0))
    };

    float inflation_radius_f = static_cast<float>(inflation_radius_m);
    float inscribed_radius_f = static_cast<float>(inscribed_radius_m);

    // Dijkstra wavefront expansion
    while (!queue.empty()) {
        Cell current = queue.top();
        queue.pop();

        // Skip if we've already found a shorter path to this cell
        if (current.distance > distance_grid(current.row, current.col)) {
            continue;
        }

        for (int i = 0; i < 8; ++i) {
            int nr = current.row + dx[i];
            int nc = current.col + dy[i];

            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;

            float new_dist = current.distance + dd[i];

            // Don't expand beyond inflation radius
            if (new_dist > inflation_radius_f) continue;

            if (new_dist < distance_grid(nr, nc)) {
                distance_grid(nr, nc) = new_dist;
                queue.push({nr, nc, new_dist});
            }
        }
    }

    // Convert distances to costs
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            float dist = distance_grid(r, c);

            if (dist >= inflation_radius_f || dist == std::numeric_limits<float>::max()) {
                inflation_data(r, c) = CostValues::FREE_SPACE;
            } else if (dist <= inscribed_radius_f) {
                inflation_data(r, c) = CostValues::INSCRIBED;
            } else {
                // Exponential decay
                float factor = static_cast<float>(
                    std::exp(-cost_scaling_factor *
                             (dist - inscribed_radius_f) /
                             (inflation_radius_f - inscribed_radius_f)));
                inflation_data(r, c) = CostValues::INSCRIBED * factor;
            }
        }
    }
}

} // namespace costs
} // namespace coastmotionplanning
