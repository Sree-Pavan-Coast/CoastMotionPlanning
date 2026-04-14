#pragma once

#include <vector>
#include <string>
#include "coastmotionplanning/zones/zone.hpp"
#include "coastmotionplanning/costs/costmap_types.hpp"

namespace coastmotionplanning {
namespace zones {

class ManeuveringZone : public Zone {
public:
    ManeuveringZone() = default;
    ~ManeuveringZone() override = default;

    ManeuveringZone(const geometry::Polygon2d& polygon, const std::optional<std::string>& name = std::nullopt);

    std::string getDefaultPlannerBehavior() const override { return ""; }

    /// Maneuvering zones activate: static_obstacles, inflation, holonomic heuristic.
    /// No lane centerline cost (free maneuvering in any direction).
    std::vector<std::string> getActiveLayers() const override {
        return {
            costs::CostmapLayerNames::STATIC_OBSTACLES,
            costs::CostmapLayerNames::INFLATION,
            costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES
        };
    }

    /// No inherent motion-direction restriction. The selected planner profile
    /// decides whether reverse is allowed for this zone type.
    bool isReverseAllowed() const override { return true; }
};

} // namespace zones
} // namespace coastmotionplanning
