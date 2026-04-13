#include "coastmotionplanning/motion_primitives/car_motion_table.hpp"
#include <cmath>
#include <stdexcept>

namespace coastmotionplanning {
namespace motion_primitives {

// ---------------------------------------------------------------------------
// Dubin: forward-only (3 base primitives + optional interpolated turns)
// ---------------------------------------------------------------------------
void CarMotionTable::initDubin(const MotionTableConfig& config) {
    model_ = Model::DUBIN;

    // Store penalties
    non_straight_penalty_    = config.non_straight_penalty;
    change_penalty_          = config.change_penalty;
    cost_penalty_            = config.cost_penalty;
    reverse_penalty_         = config.reverse_penalty;
    travel_distance_reward_  = 1.0f - config.retrospective_penalty;
    use_quadratic_cost_penalty_ = config.use_quadratic_cost_penalty;

    num_angle_quantization_       = config.num_angle_quantization;
    num_angle_quantization_float_ = static_cast<float>(num_angle_quantization_);
    min_turning_radius_           = config.minimum_turning_radius;

    // Compute bin size (radians per heading bin)
    bin_size_ = 2.0f * static_cast<float>(M_PI) / num_angle_quantization_float_;

    // Compute the minimum arc angle whose chord >= sqrt(2) (ensures we leave
    // the current grid cell on every expansion).
    //   chord = 2 * R * sin(angle/2) >= sqrt(2)
    //   => angle >= 2 * asin(sqrt(2) / (2*R))
    float angle = 2.0f * std::asin(std::sqrt(2.0f) / (2.0f * min_turning_radius_));

    // Snap up to a multiple of bin_size so heading lands exactly on a bin boundary
    float increments;
    if (angle < bin_size_) {
        increments = 1.0f;
    } else {
        increments = std::ceil(angle / bin_size_);
    }
    angle = increments * bin_size_;

    // Geometric deflections on the turning circle
    const float delta_x = min_turning_radius_ * std::sin(angle);
    const float delta_y = min_turning_radius_ - (min_turning_radius_ * std::cos(angle));
    delta_dist_ = std::hypot(delta_x, delta_y);

    // --- 3 base primitives: Straight, Left, Right ---
    projections_.clear();
    projections_.reserve(3);
    projections_.emplace_back(delta_dist_, 0.0f, 0.0f, TurnDirection::FORWARD);
    projections_.emplace_back(delta_x, delta_y, increments, TurnDirection::LEFT);
    projections_.emplace_back(delta_x, -delta_y, -increments, TurnDirection::RIGHT);

    // Optional interpolated primitives for finer angular coverage
    if (config.allow_primitive_interpolation && increments > 1.0f) {
        projections_.reserve(3 + 2 * (static_cast<unsigned int>(increments) - 1));
        for (unsigned int i = 1; i < static_cast<unsigned int>(increments); ++i) {
            const float angle_n       = static_cast<float>(i) * bin_size_;
            const float turning_rad_n = delta_dist_ / (2.0f * std::sin(angle_n / 2.0f));
            const float delta_x_n     = turning_rad_n * std::sin(angle_n);
            const float delta_y_n     = turning_rad_n - (turning_rad_n * std::cos(angle_n));
            projections_.emplace_back(delta_x_n,  delta_y_n,  static_cast<float>(i), TurnDirection::LEFT);
            projections_.emplace_back(delta_x_n, -delta_y_n, -static_cast<float>(i), TurnDirection::RIGHT);
        }
    }

    precomputeDeltas();
}

// ---------------------------------------------------------------------------
// Reeds-Shepp: forward + reverse (6 base primitives + optional interpolated)
// ---------------------------------------------------------------------------
void CarMotionTable::initReedsShepp(const MotionTableConfig& config) {
    model_ = Model::REEDS_SHEPP;

    non_straight_penalty_    = config.non_straight_penalty;
    change_penalty_          = config.change_penalty;
    cost_penalty_            = config.cost_penalty;
    reverse_penalty_         = config.reverse_penalty;
    travel_distance_reward_  = 1.0f - config.retrospective_penalty;
    use_quadratic_cost_penalty_ = config.use_quadratic_cost_penalty;

    num_angle_quantization_       = config.num_angle_quantization;
    num_angle_quantization_float_ = static_cast<float>(num_angle_quantization_);
    min_turning_radius_           = config.minimum_turning_radius;

    bin_size_ = 2.0f * static_cast<float>(M_PI) / num_angle_quantization_float_;

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

    // --- 6 base primitives ---
    projections_.clear();
    projections_.reserve(6);
    projections_.emplace_back( delta_dist_,  0.0f,      0.0f,         TurnDirection::FORWARD);
    projections_.emplace_back( delta_x,      delta_y,   increments,   TurnDirection::LEFT);
    projections_.emplace_back( delta_x,     -delta_y,  -increments,   TurnDirection::RIGHT);
    projections_.emplace_back(-delta_dist_,  0.0f,      0.0f,         TurnDirection::REVERSE);
    projections_.emplace_back(-delta_x,      delta_y,  -increments,   TurnDirection::REV_LEFT);
    projections_.emplace_back(-delta_x,     -delta_y,   increments,   TurnDirection::REV_RIGHT);

    // Optional interpolated primitives
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

    precomputeDeltas();
}

// ---------------------------------------------------------------------------
// Shared: rotate every base primitive into every heading bin and cache
// ---------------------------------------------------------------------------
void CarMotionTable::precomputeDeltas() {
    const auto num_prims = projections_.size();

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

            // Rotate the primitive's local (x,y) into the heading frame
            delta_xs_[i][j] = projections_[i].x * cos_theta - projections_[i].y * sin_theta;
            delta_ys_[i][j] = projections_[i].x * sin_theta + projections_[i].y * cos_theta;
        }
    }

    // Precompute travel cost (arc-length) per primitive
    travel_costs_.resize(num_prims);
    for (std::size_t i = 0; i < num_prims; ++i) {
        const TurnDirection td = projections_[i].turn_dir;
        if (td != TurnDirection::FORWARD && td != TurnDirection::REVERSE) {
            // Turning: cost is arc-length = radius * arc_angle
            const float arc_angle   = std::abs(projections_[i].theta) * bin_size_;
            const float turning_rad = delta_dist_ / (2.0f * std::sin(arc_angle / 2.0f));
            travel_costs_[i] = turning_rad * arc_angle;
        } else {
            travel_costs_[i] = delta_dist_;
        }
    }
}

// ---------------------------------------------------------------------------
// Search-time: pure array lookup, no trig
// ---------------------------------------------------------------------------
MotionPoses CarMotionTable::getProjections(
    float node_x, float node_y, unsigned int heading_bin) const
{
    MotionPoses result;
    result.reserve(projections_.size());

    for (std::size_t i = 0; i < projections_.size(); ++i) {
        float new_heading = static_cast<float>(heading_bin) + projections_[i].theta;

        // Wrap to [0, num_angle_quantization)
        if (new_heading < 0.0f) {
            new_heading += num_angle_quantization_float_;
        }
        if (new_heading >= num_angle_quantization_float_) {
            new_heading -= num_angle_quantization_float_;
        }

        result.emplace_back(
            static_cast<float>(delta_xs_[i][heading_bin]) + node_x,
            static_cast<float>(delta_ys_[i][heading_bin]) + node_y,
            new_heading,
            projections_[i].turn_dir);
    }

    return result;
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------
unsigned int CarMotionTable::getClosestAngularBin(double theta_rad) const {
    auto bin = static_cast<unsigned int>(
        std::round(static_cast<float>(theta_rad) / bin_size_));
    return bin < num_angle_quantization_ ? bin : 0u;
}

float CarMotionTable::getAngleFromBin(unsigned int bin_idx) const {
    return static_cast<float>(bin_idx) * bin_size_;
}

float CarMotionTable::getTravelCost(unsigned int primitive_idx) const {
    if (primitive_idx >= travel_costs_.size()) {
        throw std::out_of_range("CarMotionTable::getTravelCost: index out of range");
    }
    return travel_costs_[primitive_idx];
}

}  // namespace motion_primitives
}  // namespace coastmotionplanning
