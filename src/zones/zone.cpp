#include "coastmotionplanning/zones/zone.hpp"

namespace coastmotionplanning {
namespace zones {

Zone::Zone(const geometry::Polygon2d& polygon, const std::optional<std::string>& name)
    : polygon_(polygon), name_(name) {}

std::string Zone::getResolvedPlannerBehavior() const {
    return planner_behavior_.value_or(getDefaultPlannerBehavior());
}

} // namespace zones
} // namespace coastmotionplanning
