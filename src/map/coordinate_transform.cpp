#include "coastmotionplanning/map/coordinate_transform.hpp"

namespace coastmotionplanning {
namespace map {

Eigen::Vector3d CoordinateTransform::llaToEcef(const LLA& lla) {
    double lat_rad = lla.lat * M_PI / 180.0;
    double lon_rad = lla.lon * M_PI / 180.0;
    
    double sin_lat = std::sin(lat_rad);
    double cos_lat = std::cos(lat_rad);
    double sin_lon = std::sin(lon_rad);
    double cos_lon = std::cos(lon_rad);

    double n = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

    Eigen::Vector3d ecef;
    ecef.x() = (n + lla.alt) * cos_lat * cos_lon;
    ecef.y() = (n + lla.alt) * cos_lat * sin_lon;
    ecef.z() = (n * (1.0 - WGS84_E2) + lla.alt) * sin_lat;

    return ecef;
}

Eigen::Vector3d CoordinateTransform::ecefToEnu(const Eigen::Vector3d& ecef, const LLA& origin_lla) {
    Eigen::Vector3d ecef_origin = llaToEcef(origin_lla);
    Eigen::Vector3d delta = ecef - ecef_origin;

    double lat_rad = origin_lla.lat * M_PI / 180.0;
    double lon_rad = origin_lla.lon * M_PI / 180.0;

    double sin_lat = std::sin(lat_rad);
    double cos_lat = std::cos(lat_rad);
    double sin_lon = std::sin(lon_rad);
    double cos_lon = std::cos(lon_rad);

    // Rotation matrix from ECEF to ENU
    Eigen::Matrix3d r;
    r << -sin_lon,           cos_lon,           0,
         -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
          cos_lat * cos_lon,  cos_lat * sin_lon, sin_lat;

    return r * delta;
}

Eigen::Vector3d CoordinateTransform::llaToWorld(const LLA& lla, const LLA& origin_lla, double model_metric) {
    Eigen::Vector3d enu = ecefToEnu(llaToEcef(lla), origin_lla);
    return enu * model_metric;
}

} // namespace map
} // namespace coastmotionplanning
