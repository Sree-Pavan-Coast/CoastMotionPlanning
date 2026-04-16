#pragma once

#include "coastmotionplanning/common/types.hpp"
#include "coastmotionplanning/common/math_constants.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <cmath>

namespace coastmotionplanning {
namespace geometry {

namespace bg = boost::geometry;

using Point2d = bg::model::d2::point_xy<double>;
using Polygon2d = bg::model::polygon<Point2d>;
using Box2d = bg::model::box<Point2d>;
using LineString2d = bg::model::linestring<Point2d>;

inline bool arePointsClose(const Point2d& a,
                           const Point2d& b,
                           double tolerance = common::EPSILON) {
    const double nonnegative_tolerance = std::abs(tolerance);
    return bg::comparable_distance(a, b) <=
           nonnegative_tolerance * nonnegative_tolerance;
}

} // namespace geometry
} // namespace coastmotionplanning
