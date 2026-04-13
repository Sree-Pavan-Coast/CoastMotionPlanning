#include "coastmotionplanning/robot/car.hpp"
#include <boost/geometry/strategies/transform.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <cmath>

namespace coastmotionplanning {
namespace robot {

Car::Car(double width, double wheelbase, double front_overhang, double rear_overhang)
    : RobotBase(width, wheelbase + front_overhang + rear_overhang),
      width_(width),
      wheelbase_(wheelbase),
      front_overhang_(front_overhang),
      rear_overhang_(rear_overhang) {}

geometry::Polygon2d Car::getRobotFootprint(const RobotState& state) const {
    // Generate rectangular footprint relative to rear axle
    double min_x = -rear_overhang_;
    double max_x = wheelbase_ + front_overhang_;
    double min_y = -width_ / 2.0;
    double max_y = width_ / 2.0;

    geometry::Polygon2d base_footprint;
    base_footprint.outer() = {
        geometry::Point2d(min_x, min_y),
        geometry::Point2d(max_x, min_y),
        geometry::Point2d(max_x, max_y),
        geometry::Point2d(min_x, max_y),
        geometry::Point2d(min_x, min_y)
    };

    geometry::Polygon2d transformed_footprint;
    geometry::bg::strategy::transform::translate_transformer<double, 2, 2> translate(state.x, state.y);
    geometry::bg::strategy::transform::rotate_transformer<geometry::bg::radian, double, 2, 2> rotate(state.yaw);

    // Apply rotation around origin, then translate
    geometry::Polygon2d rotated_footprint;
    geometry::bg::transform(base_footprint, rotated_footprint, rotate);
    geometry::bg::transform(rotated_footprint, transformed_footprint, translate);

    return transformed_footprint;
}

} // namespace robot
} // namespace coastmotionplanning
