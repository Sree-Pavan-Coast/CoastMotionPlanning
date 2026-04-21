#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <grid_map_core/grid_map_core.hpp>

#include "coastmotionplanning/common/profiling.hpp"
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/costs/zone_selector.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace costs {

struct CachedZoneCoverage {
    size_t zone_index{0};
    std::vector<grid_map::Index> cells;
};

struct CachedTrackCellMetadata {
    grid_map::Index index;
    float station_m{0.0f};
    float lane0_distance_m{0.0f};
    float lane0_heading_rad{0.0f};
    float lane1_distance_m{0.0f};
    float lane1_heading_rad{0.0f};
};

struct CachedTrackZoneMetadata {
    size_t zone_index{0};
    std::vector<CachedTrackCellMetadata> cells;
};

struct CachedCombinationGrid {
    std::vector<size_t> zone_indices;
    geometry::Polygon2d search_boundary;
    double resolution_m{0.0};
    grid_map::Length length;
    grid_map::Position center;
    Eigen::MatrixXf static_obstacles;
    Eigen::MatrixXf static_distance;
    std::vector<CachedZoneCoverage> zone_coverages;
    std::vector<CachedTrackZoneMetadata> track_metadata;

    size_t cellCount() const;
    grid_map::GridMap makeGrid(const std::string& frame_id = "world") const;
};

class MapLayerCache {
public:
    MapLayerCache(
        std::vector<std::shared_ptr<zones::Zone>> zones,
        ZoneConnectivityIndex connectivity,
        CostmapResolutionPolicy resolution_policy,
        double max_padding_m,
        common::ProfilingCollector* profiler = nullptr);

    const CostmapResolutionPolicy& resolutionPolicy() const { return resolution_policy_; }
    const std::vector<std::shared_ptr<zones::Zone>>& zones() const { return zones_; }
    const ZoneConnectivityIndex& connectivity() const { return connectivity_; }

    std::vector<size_t> zoneIndicesForSelection(
        const std::vector<std::shared_ptr<zones::Zone>>& selected_zones) const;

    std::shared_ptr<const CachedCombinationGrid> findCombinationGrid(
        const std::vector<size_t>& zone_indices,
        double resolution_m) const;

    double selectGuidanceResolutionM(const std::vector<size_t>& zone_indices) const;
    double selectHeuristicResolutionM(const std::vector<size_t>& zone_indices) const;
    double selectDynamicResolutionM() const;

private:
    struct CombinationRecord {
        std::vector<size_t> zone_indices;
        geometry::Polygon2d search_boundary;
        double guidance_resolution_m{0.0};
        double heuristic_resolution_m{0.0};
        std::unordered_map<std::string, std::shared_ptr<CachedCombinationGrid>> grids_by_resolution;
    };

    static std::string combinationKey(const std::vector<size_t>& zone_indices);
    static std::string resolutionKey(double resolution_m);
    static std::vector<double> uniqueResolutions(const CostmapResolutionPolicy& resolution_policy);

    std::shared_ptr<CachedCombinationGrid> buildCombinationGrid(
        const std::vector<size_t>& zone_indices,
        const geometry::Polygon2d& search_boundary,
        double resolution_m) const;
    geometry::Polygon2d buildCombinationBoundary(const std::vector<size_t>& zone_indices) const;
    double selectResolutionFromLadder(
        const geometry::Polygon2d& search_boundary,
        const std::vector<double>& ladder,
        size_t max_cells,
        double fallback_resolution_m) const;

    std::vector<std::shared_ptr<zones::Zone>> zones_;
    ZoneConnectivityIndex connectivity_;
    CostmapResolutionPolicy resolution_policy_;
    double max_padding_m{0.0};
    common::ProfilingCollector* profiler_{nullptr};
    std::unordered_map<const zones::Zone*, size_t> zone_indices_by_ptr_;
    std::unordered_map<std::string, CombinationRecord> combinations_;
};

} // namespace costs
} // namespace coastmotionplanning
