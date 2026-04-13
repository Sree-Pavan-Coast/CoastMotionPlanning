#include "coastmotionplanning/zones/track_main_road.hpp"
#include <cmath>

namespace coastmotionplanning {
namespace zones {

TrackMainRoad::TrackMainRoad(const geometry::Polygon2d& polygon, const std::optional<std::string>& name)
    : Zone(polygon, name) {}

void TrackMainRoad::setLaneWaypointsFromPoints(const std::vector<geometry::Point2d>& points) {
    lane_waypoints_.clear();
    if (points.empty()) return;

    lane_waypoints_.reserve(points.size());

    if (points.size() == 1) {
        lane_waypoints_.emplace_back(points[0].x(), points[0].y(), math::Angle::from_radians(0.0));
        return;
    }

    for (size_t i = 0; i < points.size(); ++i) {
        double yaw = 0.0;
        if (i < points.size() - 1) {
            yaw = std::atan2(points[i+1].y() - points[i].y(), points[i+1].x() - points[i].x());
        } else {
            // For the terminal point, use the orientation of the final segment
            yaw = std::atan2(points[i].y() - points[i-1].y(), points[i].x() - points[i-1].x());
        }
        lane_waypoints_.emplace_back(points[i].x(), points[i].y(), math::Angle::from_radians(yaw));
    }
}

} // namespace zones
} // namespace coastmotionplanning
