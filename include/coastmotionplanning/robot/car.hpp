#pragma once

#include "coastmotionplanning/robot/robot_base.hpp"

namespace coastmotionplanning {
namespace robot {

class Car : public RobotBase {
public:
    Car(double width, double wheelbase, double front_overhang, double rear_overhang);
    ~Car() override = default;

    geometry::Polygon2d getRobotFootprint(const RobotState& state) const override;

    double getWidth() const { return width_; }
    double getWheelbase() const { return wheelbase_; }
    double getFrontOverhang() const { return front_overhang_; }
    double getRearOverhang() const { return rear_overhang_; }

private:
    double width_;
    double wheelbase_;
    double front_overhang_;
    double rear_overhang_;
};

} // namespace robot
} // namespace coastmotionplanning
