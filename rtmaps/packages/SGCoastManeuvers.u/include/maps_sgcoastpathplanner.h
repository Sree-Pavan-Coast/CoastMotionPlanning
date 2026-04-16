#pragma once

#include <maps.hpp>
#include <maps_module.h>
#include <vector>
#include <memory>
#include <mutex>
#include "coastmotionplanning/geometry/shape_types.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include "coastmotionplanning/planning/planner_behavior_set.hpp"
#include "coastmotionplanning/robot/car.hpp"
#include "coastmotionplanning/config/robots_parser.hpp"
#include "coastmotionplanning/planning/hybrid_a_star_planner.hpp"

using namespace coastmotionplanning;
using namespace coastmotionplanning::geometry;
using namespace coastmotionplanning::math;
using namespace coastmotionplanning::planning;
using namespace coastmotionplanning::robot;
using namespace coastmotionplanning::config;

struct DestinationForManeuvers {
    Pose2d goal_pose;
    Polygon2d goal_station;

    void clear(){
        goal_pose.x = 0.0;
        goal_pose.y = 0.0;
        goal_pose.theta.reset_to_zero();
        goal_station.clear();
    }
};

struct ZoneArea {
    int id = -1;
    Polygon2d zone_area;
    std::vector<Polygon2d> static_obstacles;

    void reset_zone_area() {
            id = -1;
            zone_area.outer().clear();
            static_obstacles.clear();
    }
};

class MAPSSGCoastPathPlanner : public MAPSComponent
{
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSSGCoastPathPlanner)
    public:

    void ZoneAreaReaderThread();
    void ReadShapesDebug();
    HybridAStarPlannerResult PlanPath(const Pose2d& start);

    private:
        DestinationForManeuvers _DestinationForManeuvers;
        ZoneArea _ZoneArea;
        std::vector<Polygon2d> _DynamicObstacles;

        bool _IsZoneAreaAvailable = false;

        PlannerBehaviorSet _PlanningBehaviors;
        std::unique_ptr<Car> _Robot = nullptr;
        std::unique_ptr<CarDefinition> _CarDefinition;

        bool _PrintDebugInfo = false;

        std::mutex mtx;
};
