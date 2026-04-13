#include "coastmotionplanning/math/pose2d.hpp"
#include <cmath>

namespace coastmotionplanning {
namespace math {

bool Pose2d::operator==(const Pose2d& other) const {
    // using a small epsilon for floating point comparison
    constexpr double epsilon = 1e-6;
    return std::abs(x - other.x) < epsilon &&
           std::abs(y - other.y) < epsilon &&
           theta == other.theta;
}

} // namespace math
} // namespace coastmotionplanning
