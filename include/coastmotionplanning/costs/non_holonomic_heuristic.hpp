#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace coastmotionplanning {
namespace costs {

/// Binary file header for non-holonomic heuristic LUT
struct NHLutHeader {
    static constexpr uint32_t MAGIC = 0x4E484C54;  // "NHLT"
    static constexpr uint32_t VERSION = 1;

    uint32_t magic{MAGIC};
    uint32_t version{VERSION};
    float min_turning_radius{0.0f};
    uint32_t num_angle_bins{0};
    uint32_t grid_size{0};           // Full grid dimension (e.g. 400)
    float cell_size{0.0f};           // Resolution in meters
    uint32_t stored_y_size{0};       // Half grid: grid_size/2 + 1 (y >= 0 only)
    uint32_t reserved{0};
};

/// Precomputed non-holonomic heuristic lookup table.
/// Stores Reeds-Shepp (or Dubin) optimal path lengths from any relative
/// (dx, dy, dtheta) to the goal, ignoring obstacles.
///
/// Storage optimization: only the y >= 0 half is stored, using the
/// Reeds-Shepp symmetry L(x, y, θ) = L(x, -y, -θ).
class NonHolonomicHeuristic {
public:
    NonHolonomicHeuristic() = default;

    /// Load a precomputed LUT from a binary file.
    /// @param filepath Path to the .bin file
    /// @return true if loaded successfully
    bool loadFromFile(const std::string& filepath);

    /// Save the current LUT to a binary file.
    /// @param filepath Path to write the .bin file
    /// @return true if saved successfully
    bool saveToFile(const std::string& filepath) const;

    /// Look up the non-holonomic cost for a relative pose to goal.
    /// @param dx     Relative x displacement (meters)
    /// @param dy     Relative y displacement (meters)
    /// @param dtheta Relative heading difference (radians)
    /// @return The heuristic cost (path length in meters), or a large value if out of range
    float lookup(float dx, float dy, float dtheta) const;

    /// Look up with additional truck-trailer hitch angle penalty.
    /// @param dx              Relative x displacement (meters)
    /// @param dy              Relative y displacement (meters)
    /// @param dtheta_tractor  Relative tractor heading difference (radians)
    /// @param hitch_angle     Current hitch angle (radians)
    /// @param goal_hitch      Goal hitch angle (radians)
    /// @param penalty_factor  Weight for hitch angle penalty
    /// @return max(reeds_shepp_cost, hitch_penalty)
    float lookupWithHitchPenalty(float dx, float dy, float dtheta_tractor,
                                 float hitch_angle, float goal_hitch,
                                 float penalty_factor) const;

    /// Initialize an empty LUT grid for computation
    void initGrid(float min_turning_radius, uint32_t num_angle_bins,
                  uint32_t grid_size, float cell_size);

    /// Direct access to the data for LUT generation
    float& at(int x_idx, int y_idx, int theta_idx);
    float at(int x_idx, int y_idx, int theta_idx) const;

    // Accessors
    float getMinTurningRadius() const { return header_.min_turning_radius; }
    uint32_t getNumAngleBins() const { return header_.num_angle_bins; }
    uint32_t getGridSize() const { return header_.grid_size; }
    float getCellSize() const { return header_.cell_size; }
    bool isLoaded() const { return !data_.empty(); }

private:
    /// Convert continuous pose to grid indices (for y >= 0 only)
    bool toIndices(float dx, float dy, float dtheta,
                   int& x_idx, int& y_idx, int& theta_idx) const;

    /// Normalize angle to [0, 2π)
    static float normalizeAngle(float angle);

    NHLutHeader header_;
    std::vector<float> data_;  // Flattened 3D: [x][y_half][theta]
};

} // namespace costs
} // namespace coastmotionplanning
