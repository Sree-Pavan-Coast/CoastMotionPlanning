#pragma once

#include "coastmotionplanning/motion_primitives/motion_primitive_types.hpp"
#include <vector>
#include <cmath>

namespace coastmotionplanning {
namespace motion_primitives {

/// A pose for a truck-trailer system: tractor rear-axle (x, y, theta1)
/// plus trailer rear-axle angle theta2.  All coordinates in map-cell units;
/// angles in heading-bin increments.
struct TruckTrailerPose {
    float x{0.0f};              ///< tractor rear-axle x  (map cells)
    float y{0.0f};              ///< tractor rear-axle y  (map cells)
    float theta1{0.0f};         ///< tractor heading      (bin increments)
    float theta2{0.0f};         ///< trailer heading       (bin increments)
    TurnDirection turn_dir{TurnDirection::UNKNOWN};

    TruckTrailerPose() = default;
    TruckTrailerPose(float x_, float y_, float t1, float t2, TurnDirection td)
        : x(x_), y(y_), theta1(t1), theta2(t2), turn_dir(td) {}
};

using TruckTrailerPoses = std::vector<TruckTrailerPose>;

/// Configuration specific to the truck-trailer's kinematic coupling.
struct TruckTrailerKinematics {
    float tractor_wheelbase{3.0f};  ///< L1: tractor rear-axle to front-axle (map cells)
    float hitch_offset{1.0f};      ///< M:  tractor rear-axle to hitch      (map cells, +backward)
    float trailer_wheelbase{5.0f};  ///< L2: hitch to trailer rear-axle      (map cells)
    float max_hitch_angle{1.05f};   ///< maximum |theta1 - theta2| before jackknife (radians)
};

/// Precomputed motion table for a truck-trailer (tractor + single trailer).
///
/// State space: (x, y, theta_tractor_bin, theta_trailer_bin)
///
/// Unlike the car table which only indexes by tractor heading, this table
/// must handle a 2D heading space (tractor × trailer).  We precompute the
/// tractor's spatial deltas exactly like CarMotionTable (pure lookup), then
/// apply the trailer kinematic integration at init time for each combination
/// of (tractor_heading_bin, trailer_heading_bin, primitive).
///
/// The trailer angle update uses the standard differential equation:
///   d(theta2)/ds = (1/L2) * sin(theta1 - theta2)
///                  - (M / L2) * (1/L1) * tan(steer) * cos(theta1 - theta2)
///
/// Since we are discretizing across the hitch angle range, the per-primitive
/// trailer delta is stored in a 3D lookup:
///   trailer_delta_bins_[primitive][tractor_bin][trailer_bin]
class TruckTrailerMotionTable {
public:
    TruckTrailerMotionTable() = default;

    /// Initialize with Reeds-Shepp tractor primitives and trailer kinematics.
    /// Precomputes all lookup arrays.
    void init(const MotionTableConfig& config,
              const TruckTrailerKinematics& kinematics);

    /// Get successor poses for a given tractor + trailer heading state.
    /// Returns only primitives that do NOT violate jackknife limits.
    TruckTrailerPoses getProjections(
        float node_x, float node_y,
        unsigned int tractor_heading_bin,
        unsigned int trailer_heading_bin) const;

    /// Get the precomputed travel cost for a primitive index.
    float getTravelCost(unsigned int primitive_idx) const;

    /// Get the closest angular bin for a raw angle in radians.
    unsigned int getClosestAngularBin(double theta_rad) const;

    /// Convert a bin index back to radians.
    float getAngleFromBin(unsigned int bin_idx) const;

    // ----- Accessors -----
    unsigned int getNumPrimitives()  const { return static_cast<unsigned int>(projections_.size()); }
    unsigned int getNumAngleBins()   const { return num_angle_quantization_; }
    float        getBinSize()        const { return bin_size_; }
    float        getMinTurningRadius() const { return min_turning_radius_; }
    float        getMaxHitchAngle()  const { return kinematics_.max_hitch_angle; }

    // Penalty accessors
    float getNonStraightPenalty()   const { return non_straight_penalty_; }
    float getChangePenalty()        const { return change_penalty_; }
    float getReversePenalty()       const { return reverse_penalty_; }
    float getCostPenalty()          const { return cost_penalty_; }
    float getTravelDistanceReward() const { return travel_distance_reward_; }
    bool  getUseQuadraticCostPenalty() const { return use_quadratic_cost_penalty_; }

private:
    /// Compute the trailer's angular delta (in bin increments) for a given
    /// tractor primitive applied at a specific (tractor_heading, trailer_heading).
    float computeTrailerDelta(
        unsigned int primitive_idx,
        unsigned int tractor_heading_bin,
        unsigned int trailer_heading_bin) const;

    // Tractor base primitives (same as CarMotionTable Reeds-Shepp)
    MotionPoses projections_;

    // Precomputed tractor spatial deltas: [primitive][tractor_heading_bin]
    std::vector<std::vector<double>> delta_xs_;
    std::vector<std::vector<double>> delta_ys_;

    // Precomputed trailer angular deltas:
    // trailer_delta_bins_[primitive][tractor_bin][trailer_bin]
    std::vector<std::vector<std::vector<float>>> trailer_delta_bins_;

    // Cached trig per heading bin
    std::vector<TrigValues> trig_values_;

    // Arc-length costs per primitive
    std::vector<float> travel_costs_;

    // Configuration
    TruckTrailerKinematics kinematics_;
    unsigned int num_angle_quantization_{72};
    float num_angle_quantization_float_{72.0f};
    float min_turning_radius_{8.0f};
    float bin_size_{0.0f};
    float delta_dist_{0.0f};

    // Penalties
    float non_straight_penalty_{1.05f};
    float change_penalty_{0.0f};
    float reverse_penalty_{2.0f};
    float cost_penalty_{2.0f};
    float travel_distance_reward_{0.985f};
    bool  use_quadratic_cost_penalty_{false};
};

}  // namespace motion_primitives
}  // namespace coastmotionplanning
