#include "coastmotionplanning/math/pose2d.hpp"

#include <algorithm>
#include <cmath>

namespace coastmotionplanning {
namespace math {

bool Pose2d::operator==(const Pose2d& other) const {
    return std::abs(x - other.x) < common::EPSILON &&
           std::abs(y - other.y) < common::EPSILON &&
           theta == other.theta;
}

common::MotionDirection Pose2d::inferMotionDirectionTo(
    const Pose2d& other,
    common::MotionDirection fallback_direction) const {
    const double dx = other.x - x;
    const double dy = other.y - y;
    const double distance = std::hypot(dx, dy);
    if (distance <= common::EPSILON) {
        return fallback_direction;
    }

    const double start_projection =
        dx * std::cos(theta.radians()) + dy * std::sin(theta.radians());
    const double end_projection =
        dx * std::cos(other.theta.radians()) + dy * std::sin(other.theta.radians());
    const double mean_projection = 0.5 * (start_projection + end_projection);
    const double projection_threshold = std::max(common::EPSILON, distance * 1e-3);
    if (mean_projection > projection_threshold) {
        return common::MotionDirection::Forward;
    }
    if (mean_projection < -projection_threshold) {
        return common::MotionDirection::Reverse;
    }
    return fallback_direction;
}

} // namespace math
} // namespace coastmotionplanning
