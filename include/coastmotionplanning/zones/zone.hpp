#pragma once

#include <string>
#include <optional>
#include <vector>
#include "coastmotionplanning/geometry/shape_types.hpp"

namespace coastmotionplanning {
namespace zones {

class Zone {
public:
    Zone() = default;
    virtual ~Zone() = default;

    Zone(const geometry::Polygon2d& polygon, const std::optional<std::string>& name = std::nullopt);

    const geometry::Polygon2d& getPolygon() const { return polygon_; }
    void setPolygon(const geometry::Polygon2d& polygon) { polygon_ = polygon; }

    const std::optional<std::string>& getName() const { return name_; }
    void setName(const std::optional<std::string>& name) { name_ = name; }

    const std::optional<std::string>& getPlannerBehavior() const { return planner_behavior_; }
    void setPlannerBehavior(const std::optional<std::string>& planner_behavior) {
        planner_behavior_ = planner_behavior;
    }

    // =========================================================================
    // Zone behavior API (override in subclasses for zone-specific behavior)
    // =========================================================================

    /// Returns the costmap layer names that this zone type activates.
    /// The CostmapBuilder will only build these layers for cells inside this zone.
    virtual std::vector<std::string> getActiveLayers() const = 0;

    /// Whether reverse motion is allowed in this zone.
    virtual bool isReverseAllowed() const = 0;

protected:
    geometry::Polygon2d polygon_;
    std::optional<std::string> name_;
    std::optional<std::string> planner_behavior_;
};

} // namespace zones
} // namespace coastmotionplanning
