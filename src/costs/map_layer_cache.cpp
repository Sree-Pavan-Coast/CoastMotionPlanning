#include "coastmotionplanning/costs/map_layer_cache.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

#include "coastmotionplanning/zones/track_main_road.hpp"

namespace coastmotionplanning {
namespace costs {

namespace {

using MultiPolygon2d = geometry::bg::model::multi_polygon<geometry::Polygon2d>;

constexpr double kProjectionEpsilon = 1e-12;

struct PolylineSegment {
    geometry::Point2d start;
    geometry::Point2d end;
    double heading_rad{0.0};
    double length_sq{0.0};
    double length_m{0.0};
    double station_start_m{0.0};
};

struct PolylineProjection {
    bool valid{false};
    double distance_sq{std::numeric_limits<double>::infinity()};
    double station_m{0.0};
    double heading_rad{0.0};
};

using Clock = std::chrono::high_resolution_clock;
using Ms = std::chrono::duration<double, std::milli>;

void recordTiming(const std::string& label,
                  const Clock::time_point& start,
                  common::ProfilingCollector* profiler) {
    if (profiler == nullptr) {
        return;
    }
    const auto elapsed = std::chrono::duration_cast<Ms>(Clock::now() - start);
    profiler->record(label, elapsed.count());
}

geometry::Polygon2d correctedPolygon(geometry::Polygon2d polygon) {
    geometry::bg::correct(polygon);
    return polygon;
}

geometry::Polygon2d exactUnionBoundary(const geometry::Polygon2d& first,
                                       const geometry::Polygon2d& second) {
    MultiPolygon2d merged;
    geometry::bg::union_(correctedPolygon(first), correctedPolygon(second), merged);
    if (merged.size() != 1) {
        throw std::runtime_error(
            "MapLayerCache: directly connected zones did not produce a single union polygon.");
    }
    return correctedPolygon(merged.front());
}

std::vector<PolylineSegment> buildPolylineSegments(const std::vector<math::Pose2d>& polyline) {
    std::vector<PolylineSegment> segments;
    double station = 0.0;
    for (size_t idx = 0; idx + 1 < polyline.size(); ++idx) {
        const geometry::Point2d start(polyline[idx].x, polyline[idx].y);
        const geometry::Point2d end(polyline[idx + 1].x, polyline[idx + 1].y);
        const double dx = end.x() - start.x();
        const double dy = end.y() - start.y();
        const double length_sq = (dx * dx) + (dy * dy);
        if (length_sq <= kProjectionEpsilon) {
            continue;
        }

        const double length_m = std::sqrt(length_sq);
        segments.push_back(PolylineSegment{
            start,
            end,
            std::atan2(dy, dx),
            length_sq,
            length_m,
            station
        });
        station += length_m;
    }
    return segments;
}

double squaredDistance(const geometry::Point2d& first, const geometry::Point2d& second) {
    const double dx = first.x() - second.x();
    const double dy = first.y() - second.y();
    return (dx * dx) + (dy * dy);
}

PolylineProjection projectOntoSegments(const geometry::Point2d& point,
                                       const std::vector<PolylineSegment>& segments) {
    PolylineProjection best;
    for (const auto& segment : segments) {
        if (segment.length_sq <= kProjectionEpsilon) {
            continue;
        }

        const double px = point.x();
        const double py = point.y();
        const double x0 = segment.start.x();
        const double y0 = segment.start.y();
        const double x1 = segment.end.x();
        const double y1 = segment.end.y();
        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double t = std::clamp(
            (((px - x0) * dx) + ((py - y0) * dy)) / segment.length_sq,
            0.0,
            1.0);
        const geometry::Point2d nearest_point(x0 + (t * dx), y0 + (t * dy));
        const double distance_sq = squaredDistance(point, nearest_point);
        if (best.valid && distance_sq >= best.distance_sq) {
            continue;
        }

        best.valid = true;
        best.distance_sq = distance_sq;
        best.station_m = segment.station_start_m + (t * segment.length_m);
        best.heading_rad = segment.heading_rad;
    }
    return best;
}

void stampPolygon(grid_map::GridMap& grid,
                  const geometry::Polygon2d& polygon,
                  float fill_value,
                  const std::string& layer) {
    geometry::Polygon2d corrected_polygon = correctedPolygon(polygon);
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

    for (size_t i = 0; i < vertex_count; ++i) {
        gm_polygon.addVertex(grid_map::Position(
            geometry::bg::get<0>(outer[i]),
            geometry::bg::get<1>(outer[i])));
    }

    for (grid_map::PolygonIterator it(grid, gm_polygon); !it.isPastEnd(); ++it) {
        grid.at(layer, *it) = fill_value;
    }
}

Eigen::MatrixXf computeDistanceField(const Eigen::MatrixXf& obstacle_data,
                                     double resolution_m,
                                     double max_distance_m) {
    const int rows = obstacle_data.rows();
    const int cols = obstacle_data.cols();
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
        static_cast<float>(resolution_m * std::sqrt(2.0)),
        static_cast<float>(resolution_m),
        static_cast<float>(resolution_m * std::sqrt(2.0)),
        static_cast<float>(resolution_m),
        static_cast<float>(resolution_m),
        static_cast<float>(resolution_m * std::sqrt(2.0)),
        static_cast<float>(resolution_m),
        static_cast<float>(resolution_m * std::sqrt(2.0))
    };
    const float max_distance_f = static_cast<float>(max_distance_m);

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
            if (next_distance > max_distance_f) {
                continue;
            }
            if (next_distance >= distance_grid(next_row, next_col)) {
                continue;
            }

            distance_grid(next_row, next_col) = next_distance;
            queue.push(Cell{next_row, next_col, next_distance});
        }
    }

    return distance_grid;
}

std::vector<double> sortedUniquePositiveResolutions(std::vector<double> values) {
    values.erase(
        std::remove_if(
            values.begin(),
            values.end(),
            [](double value) { return !std::isfinite(value) || value <= 0.0; }),
        values.end());
    std::sort(values.begin(), values.end());
    values.erase(
        std::unique(
            values.begin(),
            values.end(),
            [](double lhs, double rhs) { return std::abs(lhs - rhs) < 1e-9; }),
        values.end());
    return values;
}

size_t estimateCellCount(const geometry::Polygon2d& search_boundary,
                         double resolution_m,
                         double padding_m) {
    geometry::Box2d bbox;
    geometry::bg::envelope(search_boundary, bbox);
    const double length_x =
        geometry::bg::get<geometry::bg::max_corner, 0>(bbox) -
        geometry::bg::get<geometry::bg::min_corner, 0>(bbox) +
        (2.0 * padding_m);
    const double length_y =
        geometry::bg::get<geometry::bg::max_corner, 1>(bbox) -
        geometry::bg::get<geometry::bg::min_corner, 1>(bbox) +
        (2.0 * padding_m);
    const auto rows = static_cast<size_t>(std::ceil(length_x / resolution_m));
    const auto cols = static_cast<size_t>(std::ceil(length_y / resolution_m));
    return rows * cols;
}

} // namespace

size_t CachedCombinationGrid::cellCount() const {
    return static_cast<size_t>(static_obstacles.rows()) *
           static_cast<size_t>(static_obstacles.cols());
}

grid_map::GridMap CachedCombinationGrid::makeGrid(const std::string& frame_id) const {
    grid_map::GridMap grid;
    grid.setFrameId(frame_id);
    grid.setGeometry(length, resolution_m, center);
    return grid;
}

MapLayerCache::MapLayerCache(
    std::vector<std::shared_ptr<zones::Zone>> zones,
    ZoneConnectivityIndex connectivity,
    CostmapResolutionPolicy resolution_policy,
    double max_padding_m,
    common::ProfilingCollector* profiler)
    : zones_(std::move(zones)),
      connectivity_(std::move(connectivity)),
      resolution_policy_(std::move(resolution_policy)),
      max_padding_m(std::max(max_padding_m, 0.0)),
      profiler_(profiler) {
    for (size_t index = 0; index < zones_.size(); ++index) {
        zone_indices_by_ptr_.emplace(zones_[index].get(), index);
    }

    const auto resolutions = uniqueResolutions(resolution_policy_);
    if (resolutions.empty()) {
        throw std::runtime_error("MapLayerCache requires at least one positive cached resolution.");
    }

    std::vector<std::vector<size_t>> combinations;
    combinations.reserve(zones_.size());
    for (size_t index = 0; index < zones_.size(); ++index) {
        combinations.push_back({index});
    }
    for (size_t first = 0; first < zones_.size(); ++first) {
        for (size_t second : connectivity_.adjacent_zone_indices[first]) {
            if (second <= first) {
                continue;
            }
            combinations.push_back({first, second});
        }
    }

    for (const auto& zone_indices : combinations) {
        CombinationRecord record;
        record.zone_indices = zone_indices;
        record.search_boundary = buildCombinationBoundary(zone_indices);
        record.guidance_resolution_m = selectResolutionFromLadder(
            record.search_boundary,
            resolution_policy_.guidance_resolutions_m,
            resolution_policy_.guidance_max_cells,
            resolutions.front());
        record.heuristic_resolution_m = selectResolutionFromLadder(
            record.search_boundary,
            resolution_policy_.heuristic_resolutions_m,
            resolution_policy_.heuristic_max_cells,
            resolutions.front());

        std::vector<double> selected_resolutions{
            record.guidance_resolution_m,
            record.heuristic_resolution_m
        };
        selected_resolutions = sortedUniquePositiveResolutions(std::move(selected_resolutions));
        for (double resolution_m : selected_resolutions) {
            const auto grid = buildCombinationGrid(
                zone_indices,
                record.search_boundary,
                resolution_m);
            record.grids_by_resolution.emplace(resolutionKey(resolution_m), grid);
        }
        combinations_.emplace(combinationKey(zone_indices), std::move(record));
    }
}

std::vector<size_t> MapLayerCache::zoneIndicesForSelection(
    const std::vector<std::shared_ptr<zones::Zone>>& selected_zones) const {
    std::vector<size_t> zone_indices;
    zone_indices.reserve(selected_zones.size());
    for (const auto& zone : selected_zones) {
        if (zone == nullptr) {
            continue;
        }
        const auto it = zone_indices_by_ptr_.find(zone.get());
        if (it == zone_indices_by_ptr_.end()) {
            throw std::runtime_error("MapLayerCache selection references a zone outside the cache.");
        }
        zone_indices.push_back(it->second);
    }
    std::sort(zone_indices.begin(), zone_indices.end());
    zone_indices.erase(std::unique(zone_indices.begin(), zone_indices.end()), zone_indices.end());
    return zone_indices;
}

std::shared_ptr<const CachedCombinationGrid> MapLayerCache::findCombinationGrid(
    const std::vector<size_t>& zone_indices,
    double resolution_m) const {
    const auto combination_it = combinations_.find(combinationKey(zone_indices));
    if (combination_it == combinations_.end()) {
        return nullptr;
    }
    const auto grid_it =
        combination_it->second.grids_by_resolution.find(resolutionKey(resolution_m));
    if (grid_it == combination_it->second.grids_by_resolution.end()) {
        return nullptr;
    }
    return grid_it->second;
}

double MapLayerCache::selectGuidanceResolutionM(const std::vector<size_t>& zone_indices) const {
    const auto it = combinations_.find(combinationKey(zone_indices));
    if (it == combinations_.end()) {
        return resolution_policy_.guidance_resolutions_m.empty()
            ? 0.1
            : resolution_policy_.guidance_resolutions_m.front();
    }
    return it->second.guidance_resolution_m;
}

double MapLayerCache::selectHeuristicResolutionM(const std::vector<size_t>& zone_indices) const {
    const auto it = combinations_.find(combinationKey(zone_indices));
    if (it == combinations_.end()) {
        return resolution_policy_.heuristic_resolutions_m.empty()
            ? 0.2
            : resolution_policy_.heuristic_resolutions_m.front();
    }
    return it->second.heuristic_resolution_m;
}

double MapLayerCache::selectDynamicResolutionM() const {
    const auto ladder = sortedUniquePositiveResolutions(
        resolution_policy_.dynamic_resolutions_m);
    if (ladder.empty()) {
        return 0.1;
    }

    const double area =
        resolution_policy_.dynamic_window_size_x_m *
        resolution_policy_.dynamic_window_size_y_m;
    for (double resolution_m : ladder) {
        const double cells = area / (resolution_m * resolution_m);
        if (cells <= static_cast<double>(resolution_policy_.dynamic_max_cells)) {
            return resolution_m;
        }
    }
    return ladder.back();
}

std::string MapLayerCache::combinationKey(const std::vector<size_t>& zone_indices) {
    std::ostringstream stream;
    for (size_t index = 0; index < zone_indices.size(); ++index) {
        if (index > 0) {
            stream << ",";
        }
        stream << zone_indices[index];
    }
    return stream.str();
}

std::string MapLayerCache::resolutionKey(double resolution_m) {
    return std::to_string(static_cast<int64_t>(std::llround(resolution_m * 1000000.0)));
}

std::vector<double> MapLayerCache::uniqueResolutions(
    const CostmapResolutionPolicy& resolution_policy) {
    std::vector<double> resolutions = resolution_policy.guidance_resolutions_m;
    resolutions.insert(
        resolutions.end(),
        resolution_policy.heuristic_resolutions_m.begin(),
        resolution_policy.heuristic_resolutions_m.end());
    return sortedUniquePositiveResolutions(std::move(resolutions));
}

std::shared_ptr<CachedCombinationGrid> MapLayerCache::buildCombinationGrid(
    const std::vector<size_t>& zone_indices,
    const geometry::Polygon2d& search_boundary,
    double resolution_m) const {
    const auto start_time = Clock::now();
    geometry::Box2d bbox;
    geometry::bg::envelope(search_boundary, bbox);

    double min_x = geometry::bg::get<geometry::bg::min_corner, 0>(bbox) - max_padding_m;
    double min_y = geometry::bg::get<geometry::bg::min_corner, 1>(bbox) - max_padding_m;
    double max_x = geometry::bg::get<geometry::bg::max_corner, 0>(bbox) + max_padding_m;
    double max_y = geometry::bg::get<geometry::bg::max_corner, 1>(bbox) + max_padding_m;

    const auto cached_grid = std::make_shared<CachedCombinationGrid>();
    cached_grid->zone_indices = zone_indices;
    cached_grid->search_boundary = search_boundary;
    cached_grid->resolution_m = resolution_m;
    cached_grid->length = grid_map::Length(max_x - min_x, max_y - min_y);
    cached_grid->center = grid_map::Position((min_x + max_x) * 0.5, (min_y + max_y) * 0.5);

    grid_map::GridMap grid = cached_grid->makeGrid();
    grid.add(CostmapLayerNames::STATIC_OBSTACLES, CostValues::LETHAL);
    stampPolygon(
        grid,
        search_boundary,
        CostValues::FREE_SPACE,
        CostmapLayerNames::STATIC_OBSTACLES);
    cached_grid->static_obstacles = grid[CostmapLayerNames::STATIC_OBSTACLES];
    cached_grid->static_distance = computeDistanceField(
        cached_grid->static_obstacles,
        resolution_m,
        max_padding_m);

    for (size_t zone_index : zone_indices) {
        CachedZoneCoverage coverage;
        coverage.zone_index = zone_index;
        grid_map::Polygon gm_polygon;
        const auto& outer = zones_[zone_index]->getPolygon().outer();
        const size_t vertex_count =
            geometry::arePointsClose(outer.front(), outer.back()) ? outer.size() - 1 : outer.size();
        for (size_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index) {
            gm_polygon.addVertex(grid_map::Position(
                geometry::bg::get<0>(outer[vertex_index]),
                geometry::bg::get<1>(outer[vertex_index])));
        }
        for (grid_map::PolygonIterator it(grid, gm_polygon); !it.isPastEnd(); ++it) {
            coverage.cells.push_back(*it);
        }
        cached_grid->zone_coverages.push_back(std::move(coverage));

        const auto track = std::dynamic_pointer_cast<zones::TrackMainRoad>(zones_[zone_index]);
        if (track == nullptr) {
            continue;
        }

        const auto centerline_segments = buildPolylineSegments(track->getCenterline());
        const auto& lanes = track->getLanes();
        if (centerline_segments.empty() || lanes.size() < 2) {
            continue;
        }
        const auto lane0_segments = buildPolylineSegments(lanes[0]);
        const auto lane1_segments = buildPolylineSegments(lanes[1]);
        if (lane0_segments.empty() || lane1_segments.empty()) {
            continue;
        }

        CachedTrackZoneMetadata track_metadata;
        track_metadata.zone_index = zone_index;
        const auto& zone_cells = cached_grid->zone_coverages.back().cells;
        track_metadata.cells.reserve(zone_cells.size());
        for (const auto& index : zone_cells) {
            grid_map::Position position;
            grid.getPosition(index, position);
            const geometry::Point2d cell_point(position.x(), position.y());
            const auto center_projection = projectOntoSegments(cell_point, centerline_segments);
            const auto lane0_projection = projectOntoSegments(cell_point, lane0_segments);
            const auto lane1_projection = projectOntoSegments(cell_point, lane1_segments);
            if (!center_projection.valid || !lane0_projection.valid || !lane1_projection.valid) {
                continue;
            }

            track_metadata.cells.push_back(CachedTrackCellMetadata{
                index,
                static_cast<float>(center_projection.station_m),
                static_cast<float>(std::sqrt(lane0_projection.distance_sq)),
                static_cast<float>(lane0_projection.heading_rad),
                static_cast<float>(std::sqrt(lane1_projection.distance_sq)),
                static_cast<float>(lane1_projection.heading_rad)
            });
        }
        cached_grid->track_metadata.push_back(std::move(track_metadata));
    }

    recordTiming(
        "costmap.cache_build." + combinationKey(zone_indices) + "." + resolutionKey(resolution_m),
        start_time,
        profiler_);
    return cached_grid;
}

geometry::Polygon2d MapLayerCache::buildCombinationBoundary(
    const std::vector<size_t>& zone_indices) const {
    if (zone_indices.empty()) {
        throw std::runtime_error("MapLayerCache cannot build an empty zone combination.");
    }
    if (zone_indices.size() == 1) {
        return correctedPolygon(zones_[zone_indices.front()]->getPolygon());
    }
    if (zone_indices.size() == 2) {
        return exactUnionBoundary(
            zones_[zone_indices[0]]->getPolygon(),
            zones_[zone_indices[1]]->getPolygon());
    }
    throw std::runtime_error("MapLayerCache only supports singleton and directly connected pairs.");
}

double MapLayerCache::selectResolutionFromLadder(
    const geometry::Polygon2d& search_boundary,
    const std::vector<double>& ladder,
    size_t max_cells,
    double fallback_resolution_m) const {
    const auto sorted_ladder = sortedUniquePositiveResolutions(ladder);
    if (sorted_ladder.empty()) {
        return fallback_resolution_m;
    }

    for (double resolution_m : sorted_ladder) {
        if (estimateCellCount(search_boundary, resolution_m, max_padding_m) <= max_cells) {
            return resolution_m;
        }
    }
    return sorted_ladder.back();
}

} // namespace costs
} // namespace coastmotionplanning
