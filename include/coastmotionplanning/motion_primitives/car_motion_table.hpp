#pragma once

#include "coastmotionplanning/motion_primitives/motion_primitive_types.hpp"
#include <vector>
#include <cmath>

namespace coastmotionplanning {
namespace motion_primitives {

/// Precomputed motion table for a car-like (Ackermann) robot.
///
/// Supports two kinematic models:
///   - Dubin:       forward-only (3 base primitives: straight, left, right)
///   - ReedsShepp:  forward + reverse (6 base primitives)
///
/// All trig is computed once during init(). At search time getProjections()
/// is pure array-lookup + addition — zero trig calls.
class CarMotionTable {
public:
    enum class Model { DUBIN, REEDS_SHEPP };

    CarMotionTable() = default;

    /// Initialize the table for Dubin (forward-only) kinematics.
    void initDubin(const MotionTableConfig& config);

    /// Initialize the table for Reeds-Shepp (forward + reverse) kinematics.
    void initReedsShepp(const MotionTableConfig& config);

    /// Retrieve projected successor poses for a node at (x, y, heading_bin).
    /// heading_bin is the quantized angular index [0, num_angle_quantization).
    MotionPoses getProjections(float node_x, float node_y, unsigned int heading_bin) const;

    /// Get the closest angular bin for a raw angle in radians.
    unsigned int getClosestAngularBin(double theta_rad) const;

    /// Convert a bin index back to radians.
    float getAngleFromBin(unsigned int bin_idx) const;

    /// Get the precomputed travel cost (arc-length) for a particular primitive index.
    float getTravelCost(unsigned int primitive_idx) const;

    // ----- Accessors -----
    unsigned int getNumPrimitives() const { return static_cast<unsigned int>(projections_.size()); }
    unsigned int getNumAngleBins()  const { return num_angle_quantization_; }
    float        getBinSize()       const { return bin_size_; }
    float        getMinTurningRadius() const { return min_turning_radius_; }
    Model        getModel()         const { return model_; }

    // Penalty accessors for cost computation by the A* search
    float getNonStraightPenalty()   const { return non_straight_penalty_; }
    float getChangePenalty()        const { return change_penalty_; }
    float getReversePenalty()       const { return reverse_penalty_; }
    float getCostPenalty()          const { return cost_penalty_; }
    float getTravelDistanceReward() const { return travel_distance_reward_; }
    bool  getUseQuadraticCostPenalty() const { return use_quadratic_cost_penalty_; }

private:
    /// Internal init shared by both Dubin and Reeds-Shepp.
    void precomputeDeltas();

    Model model_{Model::REEDS_SHEPP};

    // Base primitives (defined at heading = 0)
    MotionPoses projections_;

    // Precomputed rotated deltas: delta_xs_[primitive][heading_bin]
    std::vector<std::vector<double>> delta_xs_;
    std::vector<std::vector<double>> delta_ys_;

    // Cached trig per heading bin
    std::vector<TrigValues> trig_values_;

    // Precomputed arc-length cost per primitive
    std::vector<float> travel_costs_;

    // Configuration
    unsigned int num_angle_quantization_{72};
    float num_angle_quantization_float_{72.0f};
    float min_turning_radius_{8.0f};
    float bin_size_{0.0f};
    float delta_dist_{0.0f};  ///< chord length used during init

    // Penalties (stored here so the planner can query them without a separate struct)
    float non_straight_penalty_{1.05f};
    float change_penalty_{0.0f};
    float reverse_penalty_{2.0f};
    float cost_penalty_{2.0f};
    float travel_distance_reward_{0.985f};
    bool  use_quadratic_cost_penalty_{false};
};

}  // namespace motion_primitives
}  // namespace coastmotionplanning
