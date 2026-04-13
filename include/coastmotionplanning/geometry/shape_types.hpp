#pragma once

#include "coastmotionplanning/common/types.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>

namespace coastmotionplanning {
namespace geometry {

namespace bg = boost::geometry;

using Point2d = bg::model::d2::point_xy<double>;
using Polygon2d = bg::model::polygon<Point2d>;
using Box2d = bg::model::box<Point2d>;
using LineString2d = bg::model::linestring<Point2d>;

} // namespace geometry
} // namespace coastmotionplanning
