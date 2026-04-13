#pragma once

#include "coastmotionplanning/zones/zone.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"
#include "coastmotionplanning/map/coordinate_transform.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <optional>
#include <vector>
#include <string>

namespace coastmotionplanning {
namespace map {

/**
 * @brief Parser for map YAML files including coordinate transformation.
 */
class MapParser {
public:
    enum class CoordinateType {
        WORLD,
        LAT_LONG,
        LONG_LAT
    };

    /**
     * @brief Parses a map YAML file and returns a list of zone objects.
     * 
     * @param filepath Path to the YAML file.
     * @return std::vector<std::shared_ptr<zones::Zone>> The parsed zones.
     * @throws std::runtime_error If the file or required metadata for geographic coordinates is missing.
     */
    static std::vector<std::shared_ptr<zones::Zone>> parse(const std::string& filepath);

private:
    /**
     * @brief Parses the geographic origin from the YAML node.
     */
    static CoordinateTransform::LLA parseOrigin(const YAML::Node& maps_node);

    /**
     * @brief Parses the scalar model metric from the YAML node.
     */
    static double parseModelMetric(const YAML::Node& maps_node);

    /**
     * @brief Parses the coordinate type for a zone. Defaults to world coordinates.
     */
    static CoordinateType parseCoordinateType(const YAML::Node& zone_node,
                                              const std::string& zone_label);

    /**
     * @brief Processes a list of points according to the zone's coordinate type.
     */
    static std::vector<geometry::Point2d> parsePoints(const YAML::Node& points_node, 
                                                      CoordinateType coordinate_type,
                                                      const std::optional<CoordinateTransform::LLA>& origin,
                                                      double model_metric,
                                                      const std::string& context);
};

} // namespace map
} // namespace coastmotionplanning
