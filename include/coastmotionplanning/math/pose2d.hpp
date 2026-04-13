#pragma once

#include "coastmotionplanning/common/types.hpp"

#include "coastmotionplanning/math/angle.hpp"

namespace coastmotionplanning {
namespace math {

struct Pose2d {
    double x{0.0};
    double y{0.0};
    Angle theta;

    Pose2d() = default;
    Pose2d(double x_, double y_, Angle theta_)
        : x(x_), y(y_), theta(theta_) {}

    // basic equality operator
    bool operator==(const Pose2d& other) const;
};

} // namespace math
} // namespace coastmotionplanning
