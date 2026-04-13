#include "coastmotionplanning/costs/non_holonomic_heuristic.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace coastmotionplanning {
namespace costs {

namespace {
constexpr float TWO_PI = static_cast<float>(2.0 * M_PI);
constexpr float LARGE_VALUE = 1e6f;
} // namespace

float NonHolonomicHeuristic::normalizeAngle(float angle) {
    angle = std::fmod(angle, TWO_PI);
    if (angle < 0.0f) angle += TWO_PI;
    return angle;
}

void NonHolonomicHeuristic::initGrid(float min_turning_radius,
                                      uint32_t num_angle_bins,
                                      uint32_t grid_size,
                                      float cell_size) {
    header_.min_turning_radius = min_turning_radius;
    header_.num_angle_bins = num_angle_bins;
    header_.grid_size = grid_size;
    header_.cell_size = cell_size;
    header_.stored_y_size = grid_size / 2 + 1;  // y >= 0 half

    size_t total = static_cast<size_t>(grid_size) * header_.stored_y_size * num_angle_bins;
    data_.assign(total, LARGE_VALUE);
}

float& NonHolonomicHeuristic::at(int x_idx, int y_idx, int theta_idx) {
    // y_idx should be in [0, stored_y_size)
    size_t idx = static_cast<size_t>(x_idx) * header_.stored_y_size * header_.num_angle_bins +
                 static_cast<size_t>(y_idx) * header_.num_angle_bins +
                 static_cast<size_t>(theta_idx);
    return data_[idx];
}

float NonHolonomicHeuristic::at(int x_idx, int y_idx, int theta_idx) const {
    size_t idx = static_cast<size_t>(x_idx) * header_.stored_y_size * header_.num_angle_bins +
                 static_cast<size_t>(y_idx) * header_.num_angle_bins +
                 static_cast<size_t>(theta_idx);
    return data_[idx];
}

bool NonHolonomicHeuristic::toIndices(float dx, float dy, float dtheta,
                                       int& x_idx, int& y_idx, int& theta_idx) const {
    int half_grid = static_cast<int>(header_.grid_size) / 2;
    float cs = header_.cell_size;

    x_idx = static_cast<int>(std::round(dx / cs)) + half_grid;
    y_idx = static_cast<int>(std::round(dy / cs));  // y relative to center

    // Normalize angle
    float norm_theta = normalizeAngle(dtheta);
    float bin_size = TWO_PI / static_cast<float>(header_.num_angle_bins);
    theta_idx = static_cast<int>(std::round(norm_theta / bin_size)) % static_cast<int>(header_.num_angle_bins);

    // Check bounds
    if (x_idx < 0 || x_idx >= static_cast<int>(header_.grid_size)) return false;
    if (std::abs(y_idx) >= static_cast<int>(header_.stored_y_size)) return false;

    return true;
}

float NonHolonomicHeuristic::lookup(float dx, float dy, float dtheta) const {
    if (data_.empty()) return LARGE_VALUE;

    // Apply Reeds-Shepp symmetry: L(x, y, θ) = L(x, -y, -θ)
    // We only store y >= 0, so if dy < 0, flip y and negate theta
    float eff_dy = dy;
    float eff_dtheta = dtheta;
    if (eff_dy < 0.0f) {
        eff_dy = -eff_dy;
        eff_dtheta = -eff_dtheta;
    }

    int x_idx, y_idx, theta_idx;
    if (!toIndices(dx, eff_dy, eff_dtheta, x_idx, y_idx, theta_idx)) {
        return LARGE_VALUE;  // Out of LUT range
    }

    // y_idx should now be >= 0 after the symmetry flip
    if (y_idx < 0) return LARGE_VALUE;

    return at(x_idx, y_idx, theta_idx);
}

float NonHolonomicHeuristic::lookupWithHitchPenalty(
    float dx, float dy, float dtheta_tractor,
    float hitch_angle, float goal_hitch,
    float penalty_factor) const {

    float rs_cost = lookup(dx, dy, dtheta_tractor);

    // Hitch angle penalty: cost to straighten trailer from current to goal hitch angle
    // Using the minimum angular distance
    float hitch_diff = std::abs(hitch_angle - goal_hitch);
    if (hitch_diff > static_cast<float>(M_PI)) {
        hitch_diff = TWO_PI - hitch_diff;
    }
    float hitch_cost = penalty_factor * hitch_diff;

    return std::max(rs_cost, hitch_cost);
}

bool NonHolonomicHeuristic::loadFromFile(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) return false;

    // Read header
    file.read(reinterpret_cast<char*>(&header_), sizeof(NHLutHeader));
    if (!file.good()) return false;

    // Validate
    if (header_.magic != NHLutHeader::MAGIC) return false;
    if (header_.version != NHLutHeader::VERSION) return false;
    if (header_.grid_size == 0 || header_.num_angle_bins == 0) return false;

    // Read data
    size_t total = static_cast<size_t>(header_.grid_size) *
                   header_.stored_y_size * header_.num_angle_bins;
    data_.resize(total);
    file.read(reinterpret_cast<char*>(data_.data()),
              static_cast<std::streamsize>(total * sizeof(float)));

    return file.good();
}

bool NonHolonomicHeuristic::saveToFile(const std::string& filepath) const {
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) return false;

    file.write(reinterpret_cast<const char*>(&header_), sizeof(NHLutHeader));
    file.write(reinterpret_cast<const char*>(data_.data()),
               static_cast<std::streamsize>(data_.size() * sizeof(float)));

    return file.good();
}

} // namespace costs
} // namespace coastmotionplanning
