#include "coastmotionplanning/zones/maneuvering_zone.hpp"

namespace coastmotionplanning {
namespace zones {

ManeuveringZone::ManeuveringZone(const geometry::Polygon2d& polygon, const std::optional<std::string>& name)
    : Zone(polygon, name) {}

} // namespace zones
} // namespace coastmotionplanning
