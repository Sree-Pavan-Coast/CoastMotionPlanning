#pragma once

#include <cstdint>
#include <vector>

namespace coastmotionplanning {
namespace motion_primitives {

/// Direction and turn classification for each primitive.
enum class TurnDirection : uint8_t {
    FORWARD = 0,
    LEFT,
    RIGHT,
    REVERSE,
    REV_LEFT,
    REV_RIGHT,
    UNKNOWN
};

/// A single motion primitive endpoint (or intermediate pose along a sweep).
/// Coordinates are in continuous map-cell units (not meters).
/// theta is in heading-bin increments (not radians).
struct MotionPose {
    float x{0.0f};
    float y{0.0f};
    float theta{0.0f};           ///< angular change in bin increments
    TurnDirection turn_dir{TurnDirection::UNKNOWN};

    MotionPose() = default;
    MotionPose(float x_, float y_, float theta_, TurnDirection td)
        : x(x_), y(y_), theta(theta_), turn_dir(td) {}
};

using MotionPoses = std::vector<MotionPose>;

/// Cached cos/sin pair for a heading bin.
struct TrigValues {
    double cos_val{1.0};
    double sin_val{0.0};
};

/// Parameters that control how primitives are generated and costed.
struct MotionTableConfig {
    float minimum_turning_radius{8.0f};     ///< in map-cell units
    unsigned int num_angle_quantization{72}; ///< number of heading bins (e.g. 72 = 5° each)
    unsigned int size_x{0};                  ///< costmap width for index hashing

    // Cost penalties
    float non_straight_penalty{1.05f};
    float change_penalty{0.0f};
    float reverse_penalty{2.0f};
    float cost_penalty{2.0f};
    float retrospective_penalty{0.015f};
    bool  use_quadratic_cost_penalty{false};
    bool  allow_primitive_interpolation{false};
};

/// Extended motion primitive with full trajectory (for lattice / truck-trailer).
struct MotionPrimitive {
    unsigned int trajectory_id{0};
    float start_angle{0.0f};
    float end_angle{0.0f};
    float turning_radius{0.0f};
    float trajectory_length{0.0f};
    float arc_length{0.0f};
    float straight_length{0.0f};
    bool  left_turn{false};
    MotionPoses poses;     ///< full intermediate trajectory for collision sweeping
};

using MotionPrimitives = std::vector<MotionPrimitive>;

}  // namespace motion_primitives
}  // namespace coastmotionplanning
