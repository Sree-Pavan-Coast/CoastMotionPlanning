
#pragma once

#include <maps.hpp>
#include <maps_module.h>
#include <atomic>
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/geometry/shape_types.hpp"

using namespace coastmotionplanning::math;
using namespace coastmotionplanning::geometry;

class MAPSSGManeuversToAutonomousRerouter : public MAPSComponent
{
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSSGManeuversToAutonomousRerouter)

    void ProvisionalTrajMissionChecker();
    void ReRouteMissionChecker();

    std::atomic<bool> _IsProvisionalTrajMissionActive = false;
    std::atomic<bool> _IsReRouteMissionSuccess = false;

    Angle _HeadingThreshold = Angle::from_degrees(0.0);
    double _DistThreshold = 0.0;

    std::vector<Point2d> _RefPathCoords;

};
