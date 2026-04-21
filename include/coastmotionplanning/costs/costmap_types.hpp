#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace coastmotionplanning {
namespace costs {

// =============================================================================
// Cost value constants (matching ROS nav2_costmap_2d conventions)
// =============================================================================

struct CostValues {
    static constexpr float FREE_SPACE      = 0.0f;
    static constexpr float INSCRIBED        = 128.0f;
    static constexpr float LETHAL           = 254.0f;
    static constexpr float NO_INFORMATION   = 255.0f;
};

// =============================================================================
// Layer name constants
// =============================================================================

namespace CostmapLayerNames {
    constexpr const char* STATIC_OBSTACLES           = "static_obstacles";
    constexpr const char* STATIC_DISTANCE            = "static_distance";
    constexpr const char* INFLATION                  = "inflation";
    constexpr const char* ZONE_CONSTRAINTS           = "zone_constraints";
    constexpr const char* LANE_CENTERLINE_COST       = "lane_centerline_cost";
    constexpr const char* LANE_HEADING               = "lane_heading";
    constexpr const char* LANE_DISTANCE              = "lane_distance";
    constexpr const char* TRACK_STATION              = "track_station";
    constexpr const char* DYNAMIC_OBSTACLES         = "dynamic_obstacles";
    constexpr const char* DYNAMIC_INFLATION         = "dynamic_inflation";
    constexpr const char* HOLONOMIC_WITH_OBSTACLES   = "holonomic_with_obstacles";
    constexpr const char* COMBINED_COST              = "combined_cost";
} // namespace CostmapLayerNames

// =============================================================================
// Zone constraint constants (written to the zone_constraints layer)
// The zone_constraints layer stores frontier ownership indices for cells inside
// selected zones. Cells outside all zones use ZONE_NONE. The planner reads this
// layer at runtime to map a node to its zone/profile context without repeated
// polygon-within checks.
// =============================================================================

struct ZoneConstraintValues {
    static constexpr float ZONE_NONE = 255.0f;       // Cell outside all operational zones (lethal)
};

struct CostmapResolutionPolicy {
    std::vector<double> guidance_resolutions_m;
    std::vector<double> heuristic_resolutions_m;
    std::vector<double> dynamic_resolutions_m;
    size_t guidance_max_cells{2000000};
    size_t heuristic_max_cells{1000000};
    size_t dynamic_max_cells{2000000};
    double dynamic_window_size_x_m{60.0};
    double dynamic_window_size_y_m{60.0};
};

// =============================================================================
// Configuration for the costmap builder
// =============================================================================

struct CostmapConfig {
    // Grid
    double resolution{0.1};                     // meters per cell

    // Inflation
    double inflation_radius_m{0.3};             // outer inflation radius
    double inscribed_radius_m{1.0};             // robot inscribed radius (half-width)
    double cost_scaling_factor{3.0};            // exponential decay factor

    // Lane centerline
    double max_lane_cost{100.0};                // maximum cost at lane boundary
    double max_lane_half_width{5.0};            // distance at which cost saturates

    // Non-holonomic heuristic
    double hitch_angle_penalty_factor{2.0};     // weight for truck-trailer hitch angle penalty
};

} // namespace costs
} // namespace coastmotionplanning
