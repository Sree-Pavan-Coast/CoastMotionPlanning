#include "coastmotionplanning/costs/costmap_builder.hpp"

#include <chrono>
#include <iostream>
#include <cmath>

#include <boost/geometry/algorithms/envelope.hpp>

#include "coastmotionplanning/costs/zone_selector.hpp"
#include "coastmotionplanning/costs/static_obstacle_layer.hpp"
#include "coastmotionplanning/costs/inflation_layer.hpp"
#include "coastmotionplanning/costs/zone_constraints_layer.hpp"
#include "coastmotionplanning/costs/lane_centerline_layer.hpp"
#include "coastmotionplanning/costs/holonomic_obstacles_heuristic.hpp"

namespace coastmotionplanning {
namespace costs {

namespace {
using Clock = std::chrono::high_resolution_clock;
using Ms = std::chrono::duration<double, std::milli>;

void recordTiming(const std::string& label,
                  const Clock::time_point& start,
                  coastmotionplanning::common::ProfilingCollector* profiler) {
    auto elapsed = std::chrono::duration_cast<Ms>(Clock::now() - start);
    if (profiler != nullptr) {
        profiler->record(label, elapsed.count());
        return;
    }
    std::cout << "[CostmapBuilder] " << label << ": " << elapsed.count() << " ms" << std::endl;
}
} // namespace

CostmapBuilder::CostmapBuilder(
    const CostmapConfig& config,
    const std::vector<std::shared_ptr<zones::Zone>>& all_zones,
    const robot::RobotBase& robot,
    common::ProfilingCollector* profiler)
    : config_(config), all_zones_(all_zones), robot_(robot), profiler_(profiler) {}

grid_map::GridMap CostmapBuilder::build(const math::Pose2d& start,
                                         const math::Pose2d& goal) {
    auto total_start = Clock::now();

    // ---- Step 1: Zone Selection + Concave Hull ----
    auto t = Clock::now();
    ZoneSelector selector;
    auto selection = selector.select(start, goal, all_zones_, config_.alpha_shape_alpha);
    recordTiming("costmap.zone_selection", t, profiler_);

    auto costmap = build(selection, goal);
    recordTiming("costmap.total_build", total_start, profiler_);
    return costmap;
}

grid_map::GridMap CostmapBuilder::build(const ZoneSelectionResult& selection,
                                        const math::Pose2d& goal) {
    auto t = Clock::now();

    // ---- Step 2: Create GridMap with geometry matching the search boundary ----
    // Compute bounding box of the search boundary
    geometry::Box2d bbox;
    geometry::bg::envelope(selection.search_boundary, bbox);

    double min_x = geometry::bg::get<geometry::bg::min_corner, 0>(bbox);
    double min_y = geometry::bg::get<geometry::bg::min_corner, 1>(bbox);
    double max_x = geometry::bg::get<geometry::bg::max_corner, 0>(bbox);
    double max_y = geometry::bg::get<geometry::bg::max_corner, 1>(bbox);

    // Add a small margin
    double margin = config_.inflation_radius_m + config_.resolution;
    min_x -= margin;
    min_y -= margin;
    max_x += margin;
    max_y += margin;

    double length_x = max_x - min_x;
    double length_y = max_y - min_y;
    double center_x = (min_x + max_x) / 2.0;
    double center_y = (min_y + max_y) / 2.0;

    costmap_ = grid_map::GridMap();
    costmap_.setFrameId("world");
    costmap_.setGeometry(
        grid_map::Length(length_x, length_y),
        config_.resolution,
        grid_map::Position(center_x, center_y));
    recordTiming("costmap.grid_creation", t, profiler_);

    // ---- Step 3: Static Obstacles ----
    t = Clock::now();
    StaticObstacleLayer::build(costmap_, selection.search_boundary);
    recordTiming("costmap.static_obstacles_layer", t, profiler_);

    // ---- Step 4: Inflation ----
    t = Clock::now();
    double inscribed_r = std::min(config_.inscribed_radius_m, robot_.getMaxWidth() / 2.0);
    InflationLayer::build(costmap_, config_.inflation_radius_m, inscribed_r,
                          config_.cost_scaling_factor);
    recordTiming("costmap.inflation_layer", t, profiler_);

    // ---- Step 5: Zone Constraints ----
    t = Clock::now();
    ZoneConstraintsLayer::build(costmap_, selection.selected_zones,
                                 selection.search_boundary);
    recordTiming("costmap.zone_constraints_layer", t, profiler_);

    // ---- Step 6: Lane Centerline ----
    t = Clock::now();
    LaneCenterlineLayer::build(costmap_, selection.selected_zones,
                                config_.max_lane_cost, config_.max_lane_half_width);
    recordTiming("costmap.lane_centerline_layer", t, profiler_);

    // ---- Step 7: Holonomic-with-Obstacles Heuristic ----
    t = Clock::now();
    grid_map::Position goal_pos(goal.x, goal.y);
    HolonomicObstaclesHeuristic::compute(costmap_, goal_pos);
    recordTiming("costmap.holonomic_heuristic_layer", t, profiler_);

    // ---- Step 8: Combined Cost ----
    t = Clock::now();
    costmap_.add(CostmapLayerNames::COMBINED_COST, 0.0f);
    auto& combined = costmap_[CostmapLayerNames::COMBINED_COST];
    const auto& obstacles = costmap_[CostmapLayerNames::STATIC_OBSTACLES];
    const auto& inflation = costmap_[CostmapLayerNames::INFLATION];
    const auto& zone_con = costmap_[CostmapLayerNames::ZONE_CONSTRAINTS];
    const auto& lane = costmap_[CostmapLayerNames::LANE_CENTERLINE_COST];

    // combined = max(static_obstacles, zone_out_of_bounds) + inflation + lane
    const int rows = costmap_.getSize()(0);
    const int cols = costmap_.getSize()(1);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            float obs = obstacles(r, c);
            float zc = zone_con(r, c);
            // Zone ZONE_NONE is treated as lethal (outside all operational zones)
            float zone_lethal = (zc >= ZoneConstraintValues::ZONE_NONE)
                                    ? CostValues::LETHAL : 0.0f;
            float base = std::max(obs, zone_lethal);
            if (base >= CostValues::LETHAL) {
                combined(r, c) = CostValues::LETHAL;
            } else {
                combined(r, c) = base + inflation(r, c) + lane(r, c);
            }
        }
    }
    recordTiming("costmap.combined_cost_layer", t, profiler_);

    return costmap_;
}

bool CostmapBuilder::loadNonHolonomicHeuristic(const std::string& filepath) {
    return nh_heuristic_.loadFromFile(filepath);
}

} // namespace costs
} // namespace coastmotionplanning
