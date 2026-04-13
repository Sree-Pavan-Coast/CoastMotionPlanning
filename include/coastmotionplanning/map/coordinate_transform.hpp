#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace coastmotionplanning {
namespace map {

/**
 * @brief Utility for geographic coordinate transformations using WGS84.
 */
class CoordinateTransform {
public:
    struct LLA {
        double lat; // Latitude in degrees
        double lon; // Longitude in degrees
        double alt; // Altitude in meters
    };

    /**
     * @brief Converts Latitude, Longitude, Altitude (WGS84) to Earth-Centered, Earth-Fixed (ECEF) coordinates.
     */
    static Eigen::Vector3d llaToEcef(const LLA& lla);

    /**
     * @brief Converts ECEF to East-North-Up (ENU) coordinates relative to an origin LLA.
     */
    static Eigen::Vector3d ecefToEnu(const Eigen::Vector3d& ecef, const LLA& origin_lla);

    /**
     * @brief Combined transformation from LLA to world coordinates using a scalar model metric.
     * 
     * @param lla The point to transform.
     * @param origin_lla The geographic origin (ENU center).
     * @param model_metric Scalar applied after ENU projection to map metric coordinates into the
     *        model's world-coordinate units.
     */
    static Eigen::Vector3d llaToWorld(const LLA& lla, const LLA& origin_lla, double model_metric);

private:
    static constexpr double WGS84_A = 6378137.0; // Semi-major axis
    static constexpr double WGS84_F = 1.0 / 298.257223563; // Flattening
    static constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F); // Eccentricity squared
};

} // namespace map
} // namespace coastmotionplanning
