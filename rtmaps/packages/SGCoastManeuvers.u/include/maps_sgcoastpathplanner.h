#pragma once

#include <maps.hpp>
#include <maps_module.h>

#include <mutex>
#include <vector>

#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/geometry/shape_types.hpp"

using namespace coastmotionplanning;
using namespace coastmotionplanning::geometry;

struct DestinationForManeuvers {
    math::Pose2d goal_pose;
    geometry::Polygon2d goal_station;
};


struct ZoneArea {
    Polygon2d zone_area;
    std::vector<Polygon2d> static_obstacles;
};

class MAPSSGCoastPathPlanner : public MAPSComponent
{
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSSGCoastPathPlanner)

private:
    void ZoneAreaReaderThread();
    void ReadShapesDebug();

    DestinationForManeuvers _DestinationForManeuvers;
    ZoneArea _ZoneArea;
    std::vector<Polygon2d> _DynamicObstacles;
};
