#pragma once

#include <vector>
#include "coastmotionplanning/geometry/shape_types.hpp"

namespace coastmotionplanning {
namespace robot {

struct RobotState {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    std::vector<double> articulations; // e.g. hitch_angle for tuck-trailer
};

class RobotBase {
public:
    RobotBase(double max_width, double max_length)
        : max_width_(max_width), max_length_(max_length) {}
        
    virtual ~RobotBase() = default;

    virtual double getMaxWidth() const { return max_width_; }
    virtual double getMaxLength() const { return max_length_; }

    virtual geometry::Polygon2d getRobotFootprint(const RobotState& state) const = 0;

protected:
    double max_width_;
    double max_length_;
};

} // namespace robot
} // namespace coastmotionplanning
