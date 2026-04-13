#include "coastmotionplanning/robot/truck_trailer.hpp"
#include <boost/geometry/strategies/transform.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <cmath>
#include <algorithm>

namespace coastmotionplanning {
namespace robot {

TruckTrailer::TruckTrailer(const Params& params)
    : RobotBase(std::max(params.tractor_width, params.trailer_width),
                params.tractor_wheelbase + params.tractor_front_overhang + params.hitch_offset + params.trailer_wheelbase + params.trailer_rear_overhang),
      params_(params) {}

geometry::Polygon2d TruckTrailer::getRobotFootprint(const RobotState& state) const {
    // ---- 1. Create Tractor Box ----
    // Origin is tractor rear axle
    // We extend the rear to the hitch offset to guarantee overlap with the trailer.
    double trac_min_x = std::min(-params_.tractor_rear_overhang, -params_.hitch_offset);
    double trac_max_x = params_.tractor_wheelbase + params_.tractor_front_overhang;
    double trac_min_y = -params_.tractor_width / 2.0;
    double trac_max_y = params_.tractor_width / 2.0;

    geometry::Polygon2d tractor_local;
    tractor_local.outer() = {
        geometry::Point2d(trac_min_x, trac_min_y),
        geometry::Point2d(trac_max_x, trac_min_y),
        geometry::Point2d(trac_max_x, trac_max_y),
        geometry::Point2d(trac_min_x, trac_max_y),
        geometry::Point2d(trac_min_x, trac_min_y)
    };

    // ---- 2. Create Trailer Box ----
    // Origin is at the hitch point. Default alignment: trailer faces forward (X positive).
    // The trailer extends backwards from the hitch.
    // We guarantee it reaches the hitch (x=0) for intersection.
    double trail_max_x = std::max(-params_.trailer_front_overhang, 0.0);
    double trail_min_x = -params_.trailer_wheelbase - params_.trailer_rear_overhang;
    double trail_min_y = -params_.trailer_width / 2.0;
    double trail_max_y = params_.trailer_width / 2.0;

    geometry::Polygon2d trailer_local;
    trailer_local.outer() = {
        geometry::Point2d(trail_min_x, trail_min_y),
        geometry::Point2d(trail_max_x, trail_min_y),
        geometry::Point2d(trail_max_x, trail_max_y),
        geometry::Point2d(trail_min_x, trail_max_y),
        geometry::Point2d(trail_min_x, trail_min_y)
    };

    // Apply articulation angle to trailer
    double hitch_angle = state.articulations.empty() ? 0.0 : state.articulations[0];
    geometry::bg::strategy::transform::rotate_transformer<geometry::bg::radian, double, 2, 2> rotate_hitch(hitch_angle);
    geometry::Polygon2d trailer_rotated;
    geometry::bg::transform(trailer_local, trailer_rotated, rotate_hitch);

    // Translate trailer rotated frame to the hitch point in tractor frame
    geometry::bg::strategy::transform::translate_transformer<double, 2, 2> translate_hitch(-params_.hitch_offset, 0.0);
    geometry::Polygon2d trailer_in_tractor_frame;
    geometry::bg::transform(trailer_rotated, trailer_in_tractor_frame, translate_hitch);

    // ---- 3. Union Tractor and Trailer ----
    std::vector<geometry::Polygon2d> union_result;
    geometry::bg::union_(tractor_local, trailer_in_tractor_frame, union_result);

    geometry::Polygon2d unified_local;
    if (union_result.empty()) {
        unified_local = tractor_local; // fallback
    } else if (union_result.size() == 1) {
        unified_local = union_result.front();
    } else {
        // If they still don't intersect into a single polygon, compute convex hull to guarantee one polygon
        geometry::Polygon2d hull;
        // Collect points from all
        geometry::Polygon2d multi_points;
        for (const auto& pt : tractor_local.outer()) multi_points.outer().push_back(pt);
        for (const auto& pt : trailer_in_tractor_frame.outer()) multi_points.outer().push_back(pt);
        geometry::bg::convex_hull(multi_points, unified_local);
    }

    // ---- 4. Transform Unified Footprint to Global State ----
    geometry::Polygon2d final_rotated, final_footprint;
    geometry::bg::strategy::transform::rotate_transformer<geometry::bg::radian, double, 2, 2> rotate_global(state.yaw);
    geometry::bg::strategy::transform::translate_transformer<double, 2, 2> translate_global(state.x, state.y);

    geometry::bg::transform(unified_local, final_rotated, rotate_global);
    geometry::bg::transform(final_rotated, final_footprint, translate_global);

    return final_footprint;
}

} // namespace robot
} // namespace coastmotionplanning
