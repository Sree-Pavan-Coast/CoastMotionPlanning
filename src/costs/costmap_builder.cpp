#include "coastmotionplanning/costs/costmap_builder.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

#include "coastmotionplanning/costs/dynamic_obstacle_layer.hpp"
#include "coastmotionplanning/costs/holonomic_obstacles_heuristic.hpp"
#include "coastmotionplanning/costs/zone_selector.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

namespace coastmotionplanning {
namespace costs {

namespace {

using MultiLineString2d = geometry::bg::model::multi_linestring<geometry::LineString2d>;
using MultiPolygon2d = geometry::bg::model::multi_polygon<geometry::Polygon2d>;
using Clock = std::chrono::high_resolution_clock;
using Ms = std::chrono::duration<double, std::milli>;

constexpr double kProjectionEpsilon = 1e-12;
constexpr double kOverlapAreaTolerance = 1e-9;
constexpr double kCandidateDedupTolerance = 1e-6;

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

void recordTiming(const std::string& label,
                  const Clock::time_point& start,
                  coastmotionplanning::common::ProfilingCollector* profiler) {
    const auto elapsed = std::chrono::duration_cast<Ms>(Clock::now() - start);
    if (profiler != nullptr) {
        profiler->record(label, elapsed.count());
        return;
    }
    std::cout << "[CostmapBuilder] " << label << ": " << elapsed.count() << " ms" << std::endl;
}

std::vector<double> normalizedLadder(std::vector<double> values) {
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

double maxResolutionInPolicy(const CostmapResolutionPolicy& resolution_policy,
                             double fallback_resolution_m) {
    std::vector<double> all = resolution_policy.guidance_resolutions_m;
    all.insert(
        all.end(),
        resolution_policy.heuristic_resolutions_m.begin(),
        resolution_policy.heuristic_resolutions_m.end());
    all.insert(
        all.end(),
        resolution_policy.dynamic_resolutions_m.begin(),
        resolution_policy.dynamic_resolutions_m.end());
    const auto normalized = normalizedLadder(std::move(all));
    if (normalized.empty()) {
        return fallback_resolution_m;
    }
    return normalized.back();
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

PolylineProjection projectOntoSegments(
    const geometry::Point2d& point,
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

void appendUniqueCandidate(std::vector<geometry::Point2d>& candidates,
                           const geometry::Point2d& candidate) {
    const bool duplicate = std::any_of(
        candidates.begin(),
        candidates.end(),
        [&](const auto& existing) {
            return geometry::arePointsClose(existing, candidate, kCandidateDedupTolerance);
        });
    if (!duplicate) {
        candidates.push_back(candidate);
    }
}

void appendLineCandidates(std::vector<geometry::Point2d>& candidates,
                          const geometry::LineString2d& line) {
    if (line.empty()) {
        return;
    }

    for (size_t idx = 0; idx < line.size(); ++idx) {
        appendUniqueCandidate(candidates, line[idx]);
        if (idx + 1 >= line.size()) {
            continue;
        }
        if (geometry::arePointsClose(line[idx], line[idx + 1], kCandidateDedupTolerance)) {
            continue;
        }
        appendUniqueCandidate(
            candidates,
            geometry::Point2d(
                (line[idx].x() + line[idx + 1].x()) * 0.5,
                (line[idx].y() + line[idx + 1].y()) * 0.5));
    }
}

void appendPolygonCandidates(std::vector<geometry::Point2d>& candidates,
                             const geometry::Polygon2d& polygon) {
    const auto corrected = correctedPolygon(polygon);
    const auto& outer = corrected.outer();
    if (outer.size() < 2) {
        return;
    }

    const size_t unique_vertex_count =
        geometry::arePointsClose(outer.front(), outer.back(), kCandidateDedupTolerance)
            ? outer.size() - 1
            : outer.size();
    for (size_t idx = 0; idx < unique_vertex_count; ++idx) {
        appendUniqueCandidate(candidates, outer[idx]);
        const auto& next = outer[(idx + 1) % unique_vertex_count];
        if (geometry::arePointsClose(outer[idx], next, kCandidateDedupTolerance)) {
            continue;
        }
        appendUniqueCandidate(
            candidates,
            geometry::Point2d(
                (outer[idx].x() + next.x()) * 0.5,
                (outer[idx].y() + next.y()) * 0.5));
    }
}

void appendClosestBridgeCandidates(std::vector<geometry::Point2d>& candidates,
                                   const geometry::Polygon2d& current_polygon,
                                   const geometry::Polygon2d& next_polygon) {
    std::vector<geometry::Point2d> current_samples;
    std::vector<geometry::Point2d> next_samples;
    appendPolygonCandidates(current_samples, correctedPolygon(current_polygon));
    appendPolygonCandidates(next_samples, correctedPolygon(next_polygon));
    if (current_samples.empty() || next_samples.empty()) {
        return;
    }

    double best_distance_sq = std::numeric_limits<double>::infinity();
    geometry::Point2d best_current;
    geometry::Point2d best_next;
    for (const auto& current_sample : current_samples) {
        for (const auto& next_sample : next_samples) {
            const double distance_sq = squaredDistance(current_sample, next_sample);
            if (distance_sq >= best_distance_sq) {
                continue;
            }
            best_distance_sq = distance_sq;
            best_current = current_sample;
            best_next = next_sample;
        }
    }

    if (!std::isfinite(best_distance_sq)) {
        return;
    }

    appendUniqueCandidate(candidates, best_current);
    appendUniqueCandidate(
        candidates,
        geometry::Point2d(
            (best_current.x() + best_next.x()) * 0.5,
            (best_current.y() + best_next.y()) * 0.5));
    appendUniqueCandidate(candidates, best_next);
}

std::vector<geometry::Point2d> deriveHandoffCandidates(
    const geometry::Polygon2d& current_polygon,
    const geometry::Polygon2d& next_polygon) {
    std::vector<geometry::Point2d> candidates;

    MultiPolygon2d overlap;
    geometry::bg::intersection(
        correctedPolygon(current_polygon),
        correctedPolygon(next_polygon),
        overlap);

    double overlap_area = 0.0;
    for (const auto& polygon : overlap) {
        overlap_area += std::abs(geometry::bg::area(polygon));
    }
    if (overlap_area > kOverlapAreaTolerance) {
        for (const auto& polygon : overlap) {
            appendPolygonCandidates(candidates, polygon);
        }
        return candidates;
    }

    MultiLineString2d shared_boundary;
    geometry::bg::intersection(
        boundaryLineString(correctedPolygon(current_polygon)),
        boundaryLineString(correctedPolygon(next_polygon)),
        shared_boundary);
    for (const auto& line : shared_boundary) {
        appendLineCandidates(candidates, line);
    }
    if (candidates.empty()) {
        appendClosestBridgeCandidates(candidates, current_polygon, next_polygon);
    }
    return candidates;
}

const SearchFrontierDescriptor* findOwningFrontier(
    const geometry::Point2d& point,
    const ZoneSelectionResult& selection) {
    const SearchFrontierDescriptor* owner = nullptr;
    for (const auto& frontier : selection.frontiers) {
        if (frontier.zone == nullptr) {
            continue;
        }
        if (ZoneSelector::isInsidePolygon(point, frontier.zone->getPolygon())) {
            owner = &frontier;
        }
    }
    return owner;
}

TrackLaneGuidance buildGuidanceForTrackFrontier(
    const SearchFrontierDescriptor& frontier,
    const ZoneSelectionResult& selection,
    const math::Pose2d& goal,
    const std::vector<PolylineSegment>& centerline_segments) {
    const geometry::Point2d goal_point(goal.x, goal.y);
    TrackLaneGuidance guidance;
    guidance.frontier_id = frontier.frontier_id;
    guidance.target_kind = TrackLaneGuidanceTargetKind::Goal;

    const SearchFrontierDescriptor* goal_owner = findOwningFrontier(goal_point, selection);
    geometry::Point2d target_point = goal_point;
    if (goal_owner == nullptr || goal_owner->frontier_id != frontier.frontier_id) {
        guidance.target_kind = TrackLaneGuidanceTargetKind::Handoff;
        if (frontier.frontier_id + 1 < selection.frontiers.size()) {
            const auto& next_frontier = selection.frontiers[frontier.frontier_id + 1];
            if (next_frontier.zone != nullptr) {
                const auto candidates = deriveHandoffCandidates(
                    frontier.zone->getPolygon(),
                    next_frontier.zone->getPolygon());
                if (!candidates.empty()) {
                    target_point = *std::min_element(
                        candidates.begin(),
                        candidates.end(),
                        [&](const auto& lhs, const auto& rhs) {
                            return squaredDistance(lhs, goal_point) <
                                   squaredDistance(rhs, goal_point);
                        });
                }
            }
        }
    }

    const auto target_projection = projectOntoSegments(target_point, centerline_segments);
    if (target_projection.valid) {
        guidance.target_station = target_projection.station_m;
    }
    return guidance;
}

void stampPolygon(grid_map::GridMap& costmap,
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
        geometry::arePointsClose(outer.front(), outer.back()) ? outer.size() - 1 : outer.size();
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

float inflationCostFromDistance(float distance_m,
                                double inflation_radius_m,
                                double inscribed_radius_m,
                                double cost_scaling_factor) {
    if (!std::isfinite(distance_m) || distance_m >= inflation_radius_m) {
        return CostValues::FREE_SPACE;
    }
    if (distance_m <= inscribed_radius_m) {
        return CostValues::INSCRIBED;
    }
    if (inflation_radius_m <= inscribed_radius_m + common::EPSILON) {
        return CostValues::FREE_SPACE;
    }

    const float factor = static_cast<float>(
        std::exp(-cost_scaling_factor *
                 (distance_m - static_cast<float>(inscribed_radius_m)) /
                 (inflation_radius_m - inscribed_radius_m)));
    return CostValues::INSCRIBED * factor;
}

void buildStaticInflationFromDistance(grid_map::GridMap& costmap,
                                      double inflation_radius_m,
                                      double inscribed_radius_m,
                                      double cost_scaling_factor) {
    costmap.add(CostmapLayerNames::INFLATION, CostValues::FREE_SPACE);
    auto& inflation = costmap[CostmapLayerNames::INFLATION];
    const auto& distance = costmap[CostmapLayerNames::STATIC_DISTANCE];
    for (int row = 0; row < distance.rows(); ++row) {
        for (int col = 0; col < distance.cols(); ++col) {
            inflation(row, col) = inflationCostFromDistance(
                distance(row, col),
                inflation_radius_m,
                inscribed_radius_m,
                cost_scaling_factor);
        }
    }
}

grid_map::GridMap makeDynamicGrid(const CostmapResolutionPolicy& resolution_policy,
                                  double resolution_m,
                                  const math::Pose2d& start_pose) {
    grid_map::GridMap dynamic_grid;
    dynamic_grid.setFrameId("world");
    dynamic_grid.setGeometry(
        grid_map::Length(
            resolution_policy.dynamic_window_size_x_m,
            resolution_policy.dynamic_window_size_y_m),
        resolution_m,
        grid_map::Position(start_pose.x, start_pose.y));
    return dynamic_grid;
}

grid_map::GridMap materializeMergedGuidanceCostmap(const PlanningGridBundle& bundle,
                                                   const CostmapConfig& config,
                                                   const robot::RobotBase& robot) {
    grid_map::GridMap merged = bundle.guidance_grid;
    merged.add(CostmapLayerNames::DYNAMIC_OBSTACLES, CostValues::FREE_SPACE);
    merged.add(CostmapLayerNames::DYNAMIC_INFLATION, CostValues::FREE_SPACE);
    merged.add(
        CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES,
        std::numeric_limits<float>::quiet_NaN());

    const auto& size = merged.getSize();
    for (int row = 0; row < size(0); ++row) {
        for (int col = 0; col < size(1); ++col) {
            const grid_map::Index index(row, col);
            grid_map::Position position;
            merged.getPosition(index, position);

            if (bundle.dynamic_grid.isInside(position)) {
                merged.at(CostmapLayerNames::DYNAMIC_OBSTACLES, index) =
                    bundle.dynamic_grid.atPosition(
                        CostmapLayerNames::DYNAMIC_OBSTACLES,
                        position,
                        grid_map::InterpolationMethods::INTER_NEAREST);
                merged.at(CostmapLayerNames::DYNAMIC_INFLATION, index) =
                    bundle.dynamic_grid.atPosition(
                        CostmapLayerNames::DYNAMIC_INFLATION,
                        position,
                        grid_map::InterpolationMethods::INTER_NEAREST);
            }

            if (bundle.heuristic_grid.isInside(position)) {
                merged.at(CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES, index) =
                    bundle.heuristic_grid.atPosition(
                        CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES,
                        position,
                        grid_map::InterpolationMethods::INTER_NEAREST);
            }
        }
    }

    const double inscribed_r = std::min(config.inscribed_radius_m, robot.getMaxWidth() / 2.0);
    if (!merged.exists(CostmapLayerNames::INFLATION)) {
        buildStaticInflationFromDistance(
            merged,
            config.inflation_radius_m,
            inscribed_r,
            config.cost_scaling_factor);
    }

    merged.add(CostmapLayerNames::COMBINED_COST, CostValues::FREE_SPACE);
    auto& combined = merged[CostmapLayerNames::COMBINED_COST];
    const auto& static_obstacles = merged[CostmapLayerNames::STATIC_OBSTACLES];
    const auto& dynamic_obstacles = merged[CostmapLayerNames::DYNAMIC_OBSTACLES];
    const auto& static_inflation = merged[CostmapLayerNames::INFLATION];
    const auto& dynamic_inflation = merged[CostmapLayerNames::DYNAMIC_INFLATION];
    const auto& zone_constraints = merged[CostmapLayerNames::ZONE_CONSTRAINTS];
    const auto& lane_cost = merged[CostmapLayerNames::LANE_CENTERLINE_COST];

    for (int row = 0; row < combined.rows(); ++row) {
        for (int col = 0; col < combined.cols(); ++col) {
            const float zone_owner = zone_constraints(row, col);
            const float zone_lethal =
                (std::isfinite(zone_owner) &&
                 std::abs(zone_owner - ZoneConstraintValues::ZONE_NONE) < 0.5f)
                    ? CostValues::LETHAL
                    : 0.0f;
            const float base = std::max(
                std::max(static_obstacles(row, col), dynamic_obstacles(row, col)),
                zone_lethal);
            if (base >= CostValues::LETHAL) {
                combined(row, col) = CostValues::LETHAL;
            } else {
                combined(row, col) =
                    base + static_inflation(row, col) + dynamic_inflation(row, col) + lane_cost(row, col);
            }
        }
    }

    return merged;
}

} // namespace

CostmapBuilder::CostmapBuilder(
    const CostmapConfig& config,
    const std::vector<std::shared_ptr<zones::Zone>>& all_zones,
    const robot::RobotBase& robot,
    const CostmapResolutionPolicy& resolution_policy,
    std::shared_ptr<const MapLayerCache> map_layer_cache,
    common::ProfilingCollector* profiler)
    : config_(config),
      all_zones_(all_zones),
      robot_(robot),
      resolution_policy_(resolution_policy),
      map_layer_cache_(std::move(map_layer_cache)),
      profiler_(profiler) {
    resolution_policy_.guidance_resolutions_m = normalizedLadder(
        resolution_policy_.guidance_resolutions_m.empty()
            ? std::vector<double>{config_.resolution}
            : resolution_policy_.guidance_resolutions_m);
    resolution_policy_.heuristic_resolutions_m = normalizedLadder(
        resolution_policy_.heuristic_resolutions_m.empty()
            ? std::vector<double>{config_.resolution}
            : resolution_policy_.heuristic_resolutions_m);
    resolution_policy_.dynamic_resolutions_m = normalizedLadder(
        resolution_policy_.dynamic_resolutions_m.empty()
            ? std::vector<double>{config_.resolution}
            : resolution_policy_.dynamic_resolutions_m);

    if (map_layer_cache_ == nullptr) {
        const double max_padding_m =
            config_.inflation_radius_m + maxResolutionInPolicy(resolution_policy_, config_.resolution);
        owned_map_layer_cache_ = std::make_shared<MapLayerCache>(
            all_zones_,
            ZoneSelector::buildConnectivityIndex(all_zones_),
            resolution_policy_,
            max_padding_m,
            profiler_);
        map_layer_cache_ = owned_map_layer_cache_;
    }
}

PlanningGridBundle CostmapBuilder::buildBundle(
    const math::Pose2d& start,
    const math::Pose2d& goal,
    const std::vector<geometry::Polygon2d>& obstacle_polygons) {
    const auto total_start = Clock::now();

    const auto selection_start = Clock::now();
    ZoneSelector selector;
    const auto selection = selector.select(start, goal, all_zones_);
    recordTiming("costmap.zone_selection", selection_start, profiler_);

    auto bundle = buildBundle(selection, start, goal, obstacle_polygons);
    recordTiming("costmap.total_build", total_start, profiler_);
    return bundle;
}

PlanningGridBundle CostmapBuilder::buildBundle(
    const ZoneSelectionResult& selection,
    const math::Pose2d& start,
    const math::Pose2d& goal,
    const std::vector<geometry::Polygon2d>& obstacle_polygons) {
    if (map_layer_cache_ == nullptr) {
        throw std::runtime_error("CostmapBuilder requires a valid MapLayerCache.");
    }

    track_lane_guidance_.clear();
    PlanningGridBundle bundle;
    const auto zone_indices = map_layer_cache_->zoneIndicesForSelection(selection.selected_zones);

    const double guidance_resolution_m = map_layer_cache_->selectGuidanceResolutionM(zone_indices);
    const double heuristic_resolution_m = map_layer_cache_->selectHeuristicResolutionM(zone_indices);
    const double dynamic_resolution_m = map_layer_cache_->selectDynamicResolutionM();
    bundle.guidance_resolution_m = guidance_resolution_m;
    bundle.heuristic_resolution_m = heuristic_resolution_m;
    bundle.dynamic_resolution_m = dynamic_resolution_m;

    const auto guidance_cache =
        map_layer_cache_->findCombinationGrid(zone_indices, guidance_resolution_m);
    const auto heuristic_cache =
        map_layer_cache_->findCombinationGrid(zone_indices, heuristic_resolution_m);
    if (guidance_cache == nullptr || heuristic_cache == nullptr) {
        throw std::runtime_error("CostmapBuilder failed to resolve cached combination grids.");
    }

    auto t = Clock::now();
    bundle.guidance_grid = guidance_cache->makeGrid();
    recordTiming("costmap.grid_creation", t, profiler_);

    t = Clock::now();
    bundle.guidance_grid.add(CostmapLayerNames::STATIC_OBSTACLES, CostValues::FREE_SPACE);
    bundle.guidance_grid[CostmapLayerNames::STATIC_OBSTACLES] = guidance_cache->static_obstacles;
    bundle.guidance_grid.add(CostmapLayerNames::STATIC_DISTANCE, 0.0f);
    bundle.guidance_grid[CostmapLayerNames::STATIC_DISTANCE] = guidance_cache->static_distance;
    recordTiming("costmap.static_obstacles_layer", t, profiler_);

    t = Clock::now();
    const double inscribed_r = std::min(config_.inscribed_radius_m, robot_.getMaxWidth() / 2.0);
    buildStaticInflationFromDistance(
        bundle.guidance_grid,
        config_.inflation_radius_m,
        inscribed_r,
        config_.cost_scaling_factor);
    recordTiming("costmap.inflation_layer", t, profiler_);

    t = Clock::now();
    bundle.guidance_grid.add(
        CostmapLayerNames::ZONE_CONSTRAINTS,
        ZoneConstraintValues::ZONE_NONE);
    for (const auto& frontier : selection.frontiers) {
        if (frontier.zone == nullptr) {
            continue;
        }
        const auto frontier_zone_indices =
            map_layer_cache_->zoneIndicesForSelection({frontier.zone});
        if (frontier_zone_indices.empty()) {
            continue;
        }
        const size_t zone_index = frontier_zone_indices.front();
        const auto coverage_it = std::find_if(
            guidance_cache->zone_coverages.begin(),
            guidance_cache->zone_coverages.end(),
            [&](const auto& coverage) { return coverage.zone_index == zone_index; });
        if (coverage_it == guidance_cache->zone_coverages.end()) {
            continue;
        }
        const float frontier_value = static_cast<float>(frontier.frontier_id);
        for (const auto& cell : coverage_it->cells) {
            bundle.guidance_grid.at(CostmapLayerNames::ZONE_CONSTRAINTS, cell) = frontier_value;
        }
    }
    recordTiming("costmap.zone_constraints_layer", t, profiler_);

    t = Clock::now();
    bundle.guidance_grid.add(CostmapLayerNames::LANE_CENTERLINE_COST, 0.0f);
    bundle.guidance_grid.add(
        CostmapLayerNames::LANE_HEADING,
        std::numeric_limits<float>::quiet_NaN());
    bundle.guidance_grid.add(
        CostmapLayerNames::LANE_DISTANCE,
        std::numeric_limits<float>::infinity());
    bundle.guidance_grid.add(
        CostmapLayerNames::TRACK_STATION,
        std::numeric_limits<float>::quiet_NaN());

    for (const auto& frontier : selection.frontiers) {
        auto track = std::dynamic_pointer_cast<zones::TrackMainRoad>(frontier.zone);
        if (track == nullptr) {
            continue;
        }

        const auto centerline_segments = buildPolylineSegments(track->getCenterline());
        if (centerline_segments.empty()) {
            continue;
        }
        const auto frontier_zone_indices =
            map_layer_cache_->zoneIndicesForSelection({frontier.zone});
        if (frontier_zone_indices.empty()) {
            continue;
        }
        const size_t zone_index = frontier_zone_indices.front();
        const auto track_metadata_it = std::find_if(
            guidance_cache->track_metadata.begin(),
            guidance_cache->track_metadata.end(),
            [&](const auto& metadata) { return metadata.zone_index == zone_index; });
        if (track_metadata_it == guidance_cache->track_metadata.end()) {
            continue;
        }

        const auto guidance =
            buildGuidanceForTrackFrontier(frontier, selection, goal, centerline_segments);
        bundle.track_lane_guidance.push_back(guidance);
        for (const auto& cell : track_metadata_it->cells) {
            const float frontier_owner =
                bundle.guidance_grid.at(CostmapLayerNames::ZONE_CONSTRAINTS, cell.index);
            if (std::isnan(frontier_owner) ||
                std::abs(frontier_owner - static_cast<float>(frontier.frontier_id)) >= 0.5f) {
                continue;
            }

            const bool prefer_lane0 =
                guidance.target_station >= static_cast<double>(cell.station_m);
            const float lane_distance =
                prefer_lane0 ? cell.lane0_distance_m : cell.lane1_distance_m;
            const float lane_heading =
                prefer_lane0 ? cell.lane0_heading_rad : cell.lane1_heading_rad;
            const double normalized =
                config_.max_lane_half_width > 0.0
                    ? std::min(
                          static_cast<double>(lane_distance) / config_.max_lane_half_width,
                          1.0)
                    : 0.0;

            bundle.guidance_grid.at(CostmapLayerNames::TRACK_STATION, cell.index) =
                cell.station_m;
            bundle.guidance_grid.at(CostmapLayerNames::LANE_DISTANCE, cell.index) =
                lane_distance;
            bundle.guidance_grid.at(CostmapLayerNames::LANE_HEADING, cell.index) =
                lane_heading;
            bundle.guidance_grid.at(CostmapLayerNames::LANE_CENTERLINE_COST, cell.index) =
                static_cast<float>(normalized * config_.max_lane_cost);
        }
    }
    recordTiming("costmap.lane_centerline_layer", t, profiler_);
    track_lane_guidance_ = bundle.track_lane_guidance;

    t = Clock::now();
    bundle.dynamic_grid = makeDynamicGrid(resolution_policy_, dynamic_resolution_m, start);
    DynamicObstacleLayer::build(
        bundle.dynamic_grid,
        obstacle_polygons,
        config_.inflation_radius_m,
        inscribed_r,
        config_.cost_scaling_factor);
    recordTiming("costmap.dynamic_obstacle_layer", t, profiler_);

    t = Clock::now();
    bundle.heuristic_grid = heuristic_cache->makeGrid();
    bundle.heuristic_grid.add(CostmapLayerNames::STATIC_OBSTACLES, CostValues::FREE_SPACE);
    bundle.heuristic_grid[CostmapLayerNames::STATIC_OBSTACLES] = heuristic_cache->static_obstacles;
    for (const auto& obstacle_polygon : obstacle_polygons) {
        stampPolygon(
            bundle.heuristic_grid,
            obstacle_polygon,
            CostValues::LETHAL,
            CostmapLayerNames::STATIC_OBSTACLES);
    }
    HolonomicObstaclesHeuristic::compute(
        bundle.heuristic_grid,
        grid_map::Position(goal.x, goal.y));
    recordTiming("costmap.holonomic_heuristic_layer", t, profiler_);

    return bundle;
}

grid_map::GridMap CostmapBuilder::build(const math::Pose2d& start,
                                        const math::Pose2d& goal,
                                        const std::vector<geometry::Polygon2d>& obstacle_polygons) {
    const auto bundle = buildBundle(start, goal, obstacle_polygons);
    costmap_ = materializeMergedGuidanceCostmap(bundle, config_, robot_);
    return costmap_;
}

grid_map::GridMap CostmapBuilder::build(const ZoneSelectionResult& selection,
                                        const math::Pose2d& start,
                                        const math::Pose2d& goal,
                                        const std::vector<geometry::Polygon2d>& obstacle_polygons) {
    const auto bundle = buildBundle(selection, start, goal, obstacle_polygons);
    auto combined_start = Clock::now();
    costmap_ = materializeMergedGuidanceCostmap(bundle, config_, robot_);
    recordTiming("costmap.combined_cost_layer", combined_start, profiler_);
    return costmap_;
}

bool CostmapBuilder::loadNonHolonomicHeuristic(const std::string& filepath) {
    return nh_heuristic_.loadFromFile(filepath);
}

} // namespace costs
} // namespace coastmotionplanning
