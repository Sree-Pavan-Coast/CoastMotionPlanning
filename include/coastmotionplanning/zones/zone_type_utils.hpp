#pragma once

#include <memory>
#include <string>

#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"

namespace coastmotionplanning {
namespace zones {

inline std::string zoneTypeName(const std::shared_ptr<Zone>& zone) {
    if (zone == nullptr) {
        return "";
    }
    if (std::dynamic_pointer_cast<TrackMainRoad>(zone) != nullptr) {
        return "TrackMainRoad";
    }
    if (std::dynamic_pointer_cast<ManeuveringZone>(zone) != nullptr) {
        return "ManeuveringZone";
    }
    return "Zone";
}

inline bool isSupportedZoneTypeName(const std::string& zone_type_name) {
    return zone_type_name == "ManeuveringZone" ||
           zone_type_name == "TrackMainRoad";
}

} // namespace zones
} // namespace coastmotionplanning
