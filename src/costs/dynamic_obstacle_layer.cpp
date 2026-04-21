#include "coastmotionplanning/costs/dynamic_obstacle_layer.hpp"

#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace coastmotionplanning {
namespace costs {

namespace {

void stampPolygon(grid_map::GridMap& costmap,
                  const geometry::Polygon2d& polygon,
                  float fill_value,
                  const std::string& layer) {
    geometry::Polygon2d corrected_polygon = polygon;
    geometry::bg::correct(corrected_polygon);
    if (corrected_polygon.outer().size() < 4 ||
        std::abs(geometry::bg::area(corrected_polygon)) < 1e-9) {
        return;
    }

    grid_map::Polygon gm_polygon;
    const auto& outer = corrected_polygon.outer();
    const size_t vertex_count =
        geometry::arePointsClose(outer.front(), outer.back())
            ? outer.size() - 1
            : outer.size();
    if (vertex_count < 3) {
        return;
    }

    for (size_t idx = 0; idx < vertex_count; ++idx) {
        gm_polygon.addVertex(grid_map::Position(
            geometry::bg::get<0>(outer[idx]),
            geometry::bg::get<1>(outer[idx])));
    }

    for (grid_map::PolygonIterator it(costmap, gm_polygon); !it.isPastEnd(); ++it) {
        costmap.at(layer, *it) = fill_value;
    }
}

} // namespace

void DynamicObstacleLayer::build(grid_map::GridMap& costmap,
                                 const std::vector<geometry::Polygon2d>& obstacle_polygons,
                                 double inflation_radius_m,
                                 double inscribed_radius_m,
                                 double cost_scaling_factor) {
    costmap.add(CostmapLayerNames::DYNAMIC_OBSTACLES, CostValues::FREE_SPACE);
    for (const auto& obstacle_polygon : obstacle_polygons) {
        stampPolygon(
            costmap,
            obstacle_polygon,
            CostValues::LETHAL,
            CostmapLayerNames::DYNAMIC_OBSTACLES);
    }

    costmap.add(CostmapLayerNames::DYNAMIC_INFLATION, CostValues::FREE_SPACE);
    auto& obstacle_data = costmap[CostmapLayerNames::DYNAMIC_OBSTACLES];
    auto& inflation_data = costmap[CostmapLayerNames::DYNAMIC_INFLATION];

    const auto& size = costmap.getSize();
    const int rows = size(0);
    const int cols = size(1);
    const double resolution = costmap.getResolution();

    Eigen::MatrixXf distance_grid(rows, cols);
    distance_grid.setConstant(std::numeric_limits<float>::max());

    struct Cell {
        int row;
        int col;
        float distance;
        bool operator>(const Cell& other) const { return distance > other.distance; }
    };
    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> queue;

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (obstacle_data(row, col) >= CostValues::LETHAL) {
                distance_grid(row, col) = 0.0f;
                queue.push(Cell{row, col, 0.0f});
            }
        }
    }

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

    const float inflation_radius_f = static_cast<float>(inflation_radius_m);
    const float inscribed_radius_f = static_cast<float>(inscribed_radius_m);

    while (!queue.empty()) {
        const Cell current = queue.top();
        queue.pop();

        if (current.distance > distance_grid(current.row, current.col)) {
            continue;
        }

        for (int idx = 0; idx < 8; ++idx) {
            const int next_row = current.row + dx[idx];
            const int next_col = current.col + dy[idx];
            if (next_row < 0 || next_row >= rows || next_col < 0 || next_col >= cols) {
                continue;
            }

            const float next_distance = current.distance + dd[idx];
            if (next_distance > inflation_radius_f ||
                next_distance >= distance_grid(next_row, next_col)) {
                continue;
            }

            distance_grid(next_row, next_col) = next_distance;
            queue.push(Cell{next_row, next_col, next_distance});
        }
    }

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const float distance = distance_grid(row, col);
            if (distance >= inflation_radius_f ||
                distance == std::numeric_limits<float>::max()) {
                inflation_data(row, col) = CostValues::FREE_SPACE;
            } else if (distance <= inscribed_radius_f) {
                inflation_data(row, col) = CostValues::INSCRIBED;
            } else {
                const float factor = static_cast<float>(
                    std::exp(-cost_scaling_factor *
                             (distance - inscribed_radius_f) /
                             (inflation_radius_f - inscribed_radius_f)));
                inflation_data(row, col) = CostValues::INSCRIBED * factor;
            }
        }
    }
}

} // namespace costs
} // namespace coastmotionplanning
