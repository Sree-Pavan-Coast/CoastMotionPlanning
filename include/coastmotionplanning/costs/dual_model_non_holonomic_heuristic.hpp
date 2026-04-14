#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace coastmotionplanning {
namespace costs {

enum class HeuristicModel : uint8_t {
    DUBINS = 0,
    REEDS_SHEPP = 1
};

struct DualModelNHLutHeader {
    static constexpr uint32_t MAGIC = 0x444E484C;  // "DNHL"
    static constexpr uint32_t VERSION = 1;

    uint32_t magic{MAGIC};
    uint32_t version{VERSION};
    float min_turning_radius{0.0f};
    uint32_t num_angle_bins{0};
    uint32_t grid_size{0};
    float cell_size{0.0f};
    uint32_t stored_y_size{0};
    uint32_t model_count{2};
};

class DualModelNonHolonomicHeuristic {
public:
    DualModelNonHolonomicHeuristic();
    ~DualModelNonHolonomicHeuristic();

    DualModelNonHolonomicHeuristic(DualModelNonHolonomicHeuristic&&) noexcept;
    DualModelNonHolonomicHeuristic& operator=(DualModelNonHolonomicHeuristic&&) noexcept;

    DualModelNonHolonomicHeuristic(const DualModelNonHolonomicHeuristic&) = delete;
    DualModelNonHolonomicHeuristic& operator=(const DualModelNonHolonomicHeuristic&) = delete;

    void configureOmpl(float min_turning_radius);
    void initGrid(float min_turning_radius, uint32_t num_angle_bins,
                  uint32_t grid_size, float cell_size);

    bool loadFromFile(const std::string& filepath);
    bool saveToFile(const std::string& filepath) const;

    float lookup(HeuristicModel model, float dx, float dy, float dtheta) const;

    float& at(HeuristicModel model, int x_idx, int y_idx, int theta_idx);
    float at(HeuristicModel model, int x_idx, int y_idx, int theta_idx) const;

    /// Sample waypoints along the optimal Reeds-Shepp or Dubins path.
    /// Returns {path_length, waypoints} where each waypoint is (x, y, yaw).
    /// Returns empty waypoints if OMPL spaces are not configured.
    std::pair<double, std::vector<std::array<double, 3>>> samplePath(
        HeuristicModel model,
        double from_x, double from_y, double from_yaw,
        double to_x, double to_y, double to_yaw,
        double step_size_m) const;

    float getMinTurningRadius() const { return header_.min_turning_radius; }
    uint32_t getNumAngleBins() const { return header_.num_angle_bins; }
    uint32_t getGridSize() const { return header_.grid_size; }
    float getCellSize() const { return header_.cell_size; }
    bool hasLookupTable() const { return table_loaded_; }
    bool hasOmplSpaces() const { return ompl_spaces_ != nullptr; }

private:
    struct OmplSpaces;

    bool toIndices(float dx, float dy, float dtheta,
                   int& x_idx, int& y_idx, int& theta_idx) const;
    static float normalizeAngle(float angle);
    size_t flatIndex(int x_idx, int y_idx, int theta_idx) const;

    std::vector<float>& dataFor(HeuristicModel model);
    const std::vector<float>& dataFor(HeuristicModel model) const;

    DualModelNHLutHeader header_;
    std::vector<float> dubins_data_;
    std::vector<float> reeds_shepp_data_;
    bool table_loaded_{false};
    std::unique_ptr<OmplSpaces> ompl_spaces_;
};

} // namespace costs
} // namespace coastmotionplanning
