#include "coastmotionplanning/motion_primitives/truck_trailer_motion_table.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace coastmotionplanning {
namespace motion_primitives {

// ---------------------------------------------------------------------------
// init — generate Reeds-Shepp tractor primitives, then precompute the trailer
//        angular delta for every (primitive, tractor_bin, trailer_bin) triple.
// ---------------------------------------------------------------------------
void TruckTrailerMotionTable::init(
    const MotionTableConfig& config,
    const TruckTrailerKinematics& kinematics)
{
    kinematics_ = kinematics;

    // Store penalties
    non_straight_penalty_       = config.non_straight_penalty;
    change_penalty_             = config.change_penalty;
    cost_penalty_               = config.cost_penalty;
    reverse_penalty_            = config.reverse_penalty;
    travel_distance_reward_     = 1.0f - config.retrospective_penalty;
    use_quadratic_cost_penalty_ = config.use_quadratic_cost_penalty;

    num_angle_quantization_       = config.num_angle_quantization;
    num_angle_quantization_float_ = static_cast<float>(num_angle_quantization_);
    min_turning_radius_           = config.minimum_turning_radius;

    bin_size_ = 2.0f * static_cast<float>(M_PI) / num_angle_quantization_float_;

    // ---- Step 1: Build tractor Reeds-Shepp base primitives ----
    // (identical logic to CarMotionTable::initReedsShepp)
    float angle = 2.0f * std::asin(std::sqrt(2.0f) / (2.0f * min_turning_radius_));
    float increments;
    if (angle < bin_size_) {
        increments = 1.0f;
    } else {
        increments = std::ceil(angle / bin_size_);
    }
    angle = increments * bin_size_;

    const float delta_x = min_turning_radius_ * std::sin(angle);
    const float delta_y = min_turning_radius_ - (min_turning_radius_ * std::cos(angle));
    delta_dist_ = std::hypot(delta_x, delta_y);

    projections_.clear();
    projections_.reserve(6);
    projections_.emplace_back( delta_dist_,  0.0f,      0.0f,        TurnDirection::FORWARD);
    projections_.emplace_back( delta_x,      delta_y,   increments,  TurnDirection::LEFT);
    projections_.emplace_back( delta_x,     -delta_y,  -increments,  TurnDirection::RIGHT);
    projections_.emplace_back(-delta_dist_,  0.0f,      0.0f,        TurnDirection::REVERSE);
    projections_.emplace_back(-delta_x,      delta_y,  -increments,  TurnDirection::REV_LEFT);
    projections_.emplace_back(-delta_x,     -delta_y,   increments,  TurnDirection::REV_RIGHT);

    if (config.allow_primitive_interpolation && increments > 1.0f) {
        projections_.reserve(6 + 4 * (static_cast<unsigned int>(increments) - 1));
        for (unsigned int i = 1; i < static_cast<unsigned int>(increments); ++i) {
            const float angle_n       = static_cast<float>(i) * bin_size_;
            const float turning_rad_n = delta_dist_ / (2.0f * std::sin(angle_n / 2.0f));
            const float delta_x_n     = turning_rad_n * std::sin(angle_n);
            const float delta_y_n     = turning_rad_n - (turning_rad_n * std::cos(angle_n));
            projections_.emplace_back( delta_x_n,  delta_y_n,  static_cast<float>(i), TurnDirection::LEFT);
            projections_.emplace_back( delta_x_n, -delta_y_n, -static_cast<float>(i), TurnDirection::RIGHT);
            projections_.emplace_back(-delta_x_n,  delta_y_n, -static_cast<float>(i), TurnDirection::REV_LEFT);
            projections_.emplace_back(-delta_x_n, -delta_y_n,  static_cast<float>(i), TurnDirection::REV_RIGHT);
        }
    }

    const auto num_prims = projections_.size();

    // ---- Step 2: Precompute tractor spatial deltas (same as car) ----
    delta_xs_.resize(num_prims);
    delta_ys_.resize(num_prims);
    trig_values_.resize(num_angle_quantization_);

    for (std::size_t i = 0; i < num_prims; ++i) {
        delta_xs_[i].resize(num_angle_quantization_);
        delta_ys_[i].resize(num_angle_quantization_);

        for (unsigned int j = 0; j < num_angle_quantization_; ++j) {
            const double cos_theta = std::cos(static_cast<double>(bin_size_) * j);
            const double sin_theta = std::sin(static_cast<double>(bin_size_) * j);

            if (i == 0) {
                trig_values_[j] = {cos_theta, sin_theta};
            }

            delta_xs_[i][j] = projections_[i].x * cos_theta - projections_[i].y * sin_theta;
            delta_ys_[i][j] = projections_[i].x * sin_theta + projections_[i].y * cos_theta;
        }
    }

    // ---- Step 3: Precompute trailer angular deltas ----
    // trailer_delta_bins_[primitive][tractor_bin][trailer_bin]
    trailer_delta_bins_.resize(num_prims);
    for (std::size_t p = 0; p < num_prims; ++p) {
        trailer_delta_bins_[p].resize(num_angle_quantization_);
        for (unsigned int t1 = 0; t1 < num_angle_quantization_; ++t1) {
            trailer_delta_bins_[p][t1].resize(num_angle_quantization_);
            for (unsigned int t2 = 0; t2 < num_angle_quantization_; ++t2) {
                trailer_delta_bins_[p][t1][t2] = computeTrailerDelta(
                    static_cast<unsigned int>(p), t1, t2);
            }
        }
    }

    // ---- Step 4: Precompute travel costs ----
    travel_costs_.resize(num_prims);
    for (std::size_t i = 0; i < num_prims; ++i) {
        const TurnDirection td = projections_[i].turn_dir;
        if (td != TurnDirection::FORWARD && td != TurnDirection::REVERSE) {
            const float arc_angle   = std::abs(projections_[i].theta) * bin_size_;
            const float turning_rad = delta_dist_ / (2.0f * std::sin(arc_angle / 2.0f));
            travel_costs_[i] = turning_rad * arc_angle;
        } else {
            travel_costs_[i] = delta_dist_;
        }
    }
}

// ---------------------------------------------------------------------------
// computeTrailerDelta — offline integration of the tractor-trailer kinematics
//
// Standard single-trailer kinematic model:
//   d(theta2)/ds = (1/L2) * sin(theta1 - theta2)
//                  - (M / (L1 * L2)) * tan(steer) * cos(theta1 - theta2)
//
// We integrate this over the chord length (delta_dist_) using a single
// Euler step.  For higher fidelity, a 4th-order Runge-Kutta could be
// substituted, but for the small step sizes used in search this is adequate.
//
// The function returns the trailer heading change in BIN INCREMENTS.
// ---------------------------------------------------------------------------
float TruckTrailerMotionTable::computeTrailerDelta(
    unsigned int primitive_idx,
    unsigned int tractor_heading_bin,
    unsigned int trailer_heading_bin) const
{
    // Convert bins to radians
    const double theta1 = static_cast<double>(tractor_heading_bin) * bin_size_;
    const double theta2 = static_cast<double>(trailer_heading_bin) * bin_size_;
    const double hitch_angle = theta1 - theta2;

    const auto& proj = projections_[primitive_idx];

    // Determine if this primitive is forward or reverse
    const bool is_reverse = (proj.turn_dir == TurnDirection::REVERSE ||
                             proj.turn_dir == TurnDirection::REV_LEFT ||
                             proj.turn_dir == TurnDirection::REV_RIGHT);
    const double sign = is_reverse ? -1.0 : 1.0;

    // Determine the effective steering curvature (1/R) for this primitive
    double curvature = 0.0;
    if (proj.turn_dir != TurnDirection::FORWARD && proj.turn_dir != TurnDirection::REVERSE) {
        // Turning primitive: curvature = theta_change_rad / arc_length
        const double arc_angle = std::abs(static_cast<double>(proj.theta)) * bin_size_;
        const double turning_rad = delta_dist_ / (2.0 * std::sin(arc_angle / 2.0));
        curvature = 1.0 / turning_rad;

        // Sign convention: LEFT turns have positive curvature
        if (proj.turn_dir == TurnDirection::RIGHT || proj.turn_dir == TurnDirection::REV_RIGHT) {
            curvature = -curvature;
        }
    }

    const double L1 = static_cast<double>(kinematics_.tractor_wheelbase);
    const double L2 = static_cast<double>(kinematics_.trailer_wheelbase);
    const double M  = static_cast<double>(kinematics_.hitch_offset);

    // d(theta2)/ds for a single step along the chord
    //   = (1/L2) * sin(hitch_angle) - (M * curvature / L2) * cos(hitch_angle)
    // The sign-adjusted traversal distance:
    const double ds = sign * static_cast<double>(delta_dist_);

    const double d_theta2 = ds * (
        (1.0 / L2) * std::sin(hitch_angle)
        - (M / L2) * curvature * std::cos(hitch_angle)
    );

    // Convert radians to bin increments
    return static_cast<float>(d_theta2 / bin_size_);
}

// ---------------------------------------------------------------------------
// getProjections — search-time: pure lookup, no trig
// ---------------------------------------------------------------------------
TruckTrailerPoses TruckTrailerMotionTable::getProjections(
    float node_x, float node_y,
    unsigned int tractor_heading_bin,
    unsigned int trailer_heading_bin) const
{
    TruckTrailerPoses result;
    result.reserve(projections_.size());

    for (std::size_t i = 0; i < projections_.size(); ++i) {
        // Tractor heading update (bin increments)
        float new_theta1 = static_cast<float>(tractor_heading_bin) + projections_[i].theta;
        if (new_theta1 < 0.0f) new_theta1 += num_angle_quantization_float_;
        if (new_theta1 >= num_angle_quantization_float_) new_theta1 -= num_angle_quantization_float_;

        // Trailer heading update (precomputed lookup!)
        float trailer_delta = trailer_delta_bins_[i][tractor_heading_bin][trailer_heading_bin];
        float new_theta2 = static_cast<float>(trailer_heading_bin) + trailer_delta;
        if (new_theta2 < 0.0f) new_theta2 += num_angle_quantization_float_;
        if (new_theta2 >= num_angle_quantization_float_) new_theta2 -= num_angle_quantization_float_;

        // Jackknife check: if resulting hitch angle exceeds limit, skip this primitive
        const float new_hitch_angle_rad =
            std::abs(new_theta1 - new_theta2) * bin_size_;
        // Handle wrap-around for the hitch angle
        const float wrapped_hitch =
            std::min(new_hitch_angle_rad,
                     2.0f * static_cast<float>(M_PI) - new_hitch_angle_rad);

        if (wrapped_hitch > kinematics_.max_hitch_angle) {
            continue;  // pruned — would jackknife
        }

        // Tractor spatial update (precomputed lookup!)
        result.emplace_back(
            static_cast<float>(delta_xs_[i][tractor_heading_bin]) + node_x,
            static_cast<float>(delta_ys_[i][tractor_heading_bin]) + node_y,
            new_theta1,
            new_theta2,
            projections_[i].turn_dir);
    }

    return result;
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------
float TruckTrailerMotionTable::getTravelCost(unsigned int primitive_idx) const {
    if (primitive_idx >= travel_costs_.size()) {
        throw std::out_of_range("TruckTrailerMotionTable::getTravelCost: index out of range");
    }
    return travel_costs_[primitive_idx];
}

unsigned int TruckTrailerMotionTable::getClosestAngularBin(double theta_rad) const {
    auto bin = static_cast<unsigned int>(
        std::round(static_cast<float>(theta_rad) / bin_size_));
    return bin < num_angle_quantization_ ? bin : 0u;
}

float TruckTrailerMotionTable::getAngleFromBin(unsigned int bin_idx) const {
    return static_cast<float>(bin_idx) * bin_size_;
}

}  // namespace motion_primitives
}  // namespace coastmotionplanning
