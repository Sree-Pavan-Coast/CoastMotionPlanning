#include "coastmotionplanning/zones/track_main_road.hpp"
#include <cmath>

namespace coastmotionplanning {
namespace zones {

TrackMainRoad::TrackMainRoad(const geometry::Polygon2d& polygon, const std::optional<std::string>& name)
    : Zone(polygon, name) {}

void TrackMainRoad::addLaneFromPoints(const std::vector<geometry::Point2d>& points) {
    if (points.empty()) return;

    std::vector<math::Pose2d> lane_waypoints;
    lane_waypoints.reserve(points.size());

    if (points.size() == 1) {
        lane_waypoints.emplace_back(points[0].x(), points[0].y(), math::Angle::from_radians(0.0));
        lanes_.push_back(std::move(lane_waypoints));
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
        lane_waypoints.emplace_back(points[i].x(), points[i].y(), math::Angle::from_radians(yaw));
    }

    lanes_.push_back(std::move(lane_waypoints));
}

} // namespace zones
} // namespace coastmotionplanning
