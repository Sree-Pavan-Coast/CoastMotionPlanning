#pragma once

#include "coastmotionplanning/common/math_constants.hpp"
#include <cmath>

namespace coastmotionplanning {
namespace math {

class Angle {
public:
    Angle() = default;

    // Named constructors for clarity and to prevent unit mixups
    static Angle from_radians(double rad) { return Angle(rad); }
    static Angle from_degrees(double deg) { return Angle(deg * common::DEG_TO_RAD); }

    // Accessors
    double radians() const { return rad_; }
    double degrees() const { return rad_ * common::RAD_TO_DEG; }

    // Normalizes this angle in-place to the range [-pi, pi)
    void normalize() {
        rad_ = std::fmod(rad_ + common::PI, common::TWO_PI);
        if (rad_ < 0.0) {
            rad_ += common::TWO_PI;
        }
        rad_ -= common::PI;
    }

    // Returns a new normalized angle
    Angle normalized() const {
        Angle a(*this);
        a.normalize();
        return a;
    }

    inline void reset_to_zero() {
        rad_ = 0.0;
    }

    // Arithmetic operators
    Angle operator+(const Angle& other) const { return Angle(rad_ + other.rad_); }
    Angle operator-(const Angle& other) const { return Angle(rad_ - other.rad_); }
    Angle& operator+=(const Angle& other) { rad_ += other.rad_; return *this; }
    Angle& operator-=(const Angle& other) { rad_ -= other.rad_; return *this; }
    
    Angle operator*(double scalar) const { return Angle(rad_ * scalar); }
    Angle operator/(double scalar) const { return Angle(rad_ / scalar); }

    // Comparison operators
    bool operator==(const Angle& other) const { 
        constexpr double epsilon = 1e-6;
        // Compare normalized values to ensure pi == -pi is handled smoothly
        return std::abs(normalized().rad_ - other.normalized().rad_) < epsilon; 
    }

private:
    // Explicit constructor to prevent implicit double conversions
    explicit Angle(double rad) : rad_(rad) {}
    double rad_{0.0};
};

} // namespace math
} // namespace coastmotionplanning
