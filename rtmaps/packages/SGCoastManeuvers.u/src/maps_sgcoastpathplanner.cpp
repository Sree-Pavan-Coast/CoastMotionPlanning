#include "maps_sgcoastpathplanner.h"
#include <exception>
#include "coastmotionplanning/geometry/shape_types.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"

MAPS_BEGIN_INPUTS_DEFINITION(MAPSSGCoastPathPlanner)
MAPS_INPUT("iPositionSpeed", MAPS::FilterFloat64, MAPS::SamplingReader)
MAPS_INPUT("iDestinationForManeuvers", MAPS::FilterFloat64, MAPS::FifoReader)
MAPS_INPUT("iZoneArea", MAPS::FilterFloat64, MAPS::LastOrNextReader)
MAPS_INPUT("iShapesDebug", MAPS::FilterFloat64, MAPS::LastOrNextReader)
MAPS_END_INPUTS_DEFINITION

MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSSGCoastPathPlanner)
MAPS_OUTPUT("oPath", MAPS::Float64, nullptr, nullptr, 10000)
MAPS_END_OUTPUTS_DEFINITION

MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSSGCoastPathPlanner)
MAPS_PROPERTY("print_debug_info", false, false, false)
MAPS_PROPERTY_SUBTYPE("robot_params_file_path", "", false, true, MAPS::PropertySubTypeFile | MAPS::PropertySubTypeMustExist)
MAPS_PROPERTY_SUBTYPE("planning_behaviors_file_path", "", false, true, MAPS::PropertySubTypeFile | MAPS::PropertySubTypeMustExist)
MAPS_PROPERTY_ENUM("robot_name", "Dolly|Pro_XD|E_Transit", 0, false, false)
MAPS_PROPERTY_ENUM("planner_behavior_profile", "proxd_primary_profile|dolly_primary_profile", 0, false, false)
MAPS_END_PROPERTIES_DEFINITION

MAPS_BEGIN_ACTIONS_DEFINITION(MAPSSGCoastPathPlanner)
MAPS_END_ACTIONS_DEFINITION

MAPS_COMPONENT_DEFINITION(MAPSSGCoastPathPlanner,
                          "MAPSSGCoastPathPlanner",
                          "1.0",
                          128,
                          MAPS::Threaded,
                          MAPS::Threaded,
                          -1,
                          -1,
                          -1,
                          -1)

void MAPSSGCoastPathPlanner::Birth()
{
    _IsZoneAreaAvailable = false;
    const std::string planning_behaviors_file_path = GetStringProperty("planning_behaviors_file_path");
    const std::string robot_params_file_path = GetStringProperty("robot_params_file_path");
    const std::string robot_name = GetStringProperty("robot_name");
    if (planning_behaviors_file_path.empty())
    {
        ReportError("Planning behaviors file path is not set. Check the planning behaviors file path.");
        return;
    }
    _PlanningBehaviors = PlannerBehaviorSet::loadFromFile(planning_behaviors_file_path);
    _CarDefinition = std::make_unique<CarDefinition>(RobotsParser::loadCarDefinition(robot_params_file_path, robot_name));
    _Robot = std::make_unique<robot::Car>(_CarDefinition->buildCar());
    _PlanningBehaviors.overrideMotionPrimitiveConstraints(_CarDefinition->minTurningRadiusMeters(), _CarDefinition->maxSteerAngleRadians());

    CreateThread((MAPSThreadFunction)&MAPSSGCoastPathPlanner::ZoneAreaReaderThread);

    if(GetBoolProperty("print_debug_info")){
        _PrintDebugInfo = true;
    }
}

void MAPSSGCoastPathPlanner::Core()
{
    // Check if zone area is available.
    if (_IsZoneAreaAvailable)
    {
        MAPSIOElt *input_elt = StartReading(Input("iDestinationForManeuvers"));
        if (input_elt == nullptr)
            return;

        // clear previous destination for maneuevrs
        _DestinationForManeuvers.clear();

        // Parse the destination for maneuvers
        _DestinationForManeuvers.goal_pose.x = input_elt->Float64(0);
        _DestinationForManeuvers.goal_pose.y = input_elt->Float64(1);
        _DestinationForManeuvers.goal_pose.theta = Angle::from_radians(input_elt->Float64(2));

        // Parse the station polygon
        _DestinationForManeuvers.goal_station.clear();
        for (int i = 3; i < input_elt->VectorSize(); i += 2)
        {
            _DestinationForManeuvers.goal_station.outer().emplace_back(input_elt->Float64(i), input_elt->Float64(i + 1));
        }
        bg::correct(_DestinationForManeuvers.goal_station);

        // Parse starting position
        input_elt = StartReading(Input("iPositionSpeed"));
        if (input_elt == nullptr)
            return;
        if (input_elt->VectorSize() <= 3)
        {
            ReportError("Position speed input is too small. Expected x, y, speed, heading.");
            return;
        }

        // parse start pose
        Pose2d start;
        start.x = input_elt->Float64(0);
        start.y = input_elt->Float64(1);
        start.theta = Angle::from_radians(input_elt->Float64(3));

        // Parse shapes for debug
        ReadShapesDebug();

        // Plan the path
        HybridAStarPlannerResult result;


        ReportInfo("Starting planning engine");
        try{
            result = PlanPath(start);
        } catch(std::exception& e) {
            // std::string msg = "Planning threw exception: " + e.what().c_str();
            ReportError(MAPSStreamedString() << "Planning threw exception: " << e.what());
        }

        // stats of result
        if(result.success){
            ReportInfo("Planning success");
            ReportInfo(MAPSStreamedString() << "found path has " << result.poses.size() << " poses");
        }else{
            ReportInfo("Planning failed");
        }

        MAPSIOElt *ioEltOut = StartWriting(Output("oPath"));
        long size = 0;
        for (const auto &pose : result.poses)
        {
            if(pose.x == 0.0 && pose.y == 0.0) break;
            ioEltOut->Float64(size++) = pose.x;
            ioEltOut->Float64(size++) = pose.y;
            ioEltOut->Float64(size++) = pose.theta.radians();
        }
        ioEltOut->VectorSize() = size;
        ioEltOut->Timestamp() = input_elt->Timestamp();
        StopWriting(ioEltOut);
    }
}

void MAPSSGCoastPathPlanner::Death()
{
}

void MAPSSGCoastPathPlanner::ZoneAreaReaderThread()
{
    auto &input = Input("iZoneArea");
    // auto arePointsClose = [] (const Point2d& p1, const Point2d& p2) -> bool 
    // {
    //     constexpr static auto tol = 1e-12;
    //     return bg::comparable_distance(p1, p2) <= tol;
    // };

    while (!IsDying())
    {
        while (DataAvailableInFIFO(input))
        {
            MAPSIOElt *input_elt = StartReading(input);
            if (input_elt == nullptr)
            {
                break;
            }

            // check zone area data size
            const auto zone_area_size = input_elt->VectorSize();
            if (zone_area_size < 3)
            {
                ReportError("Zone area data too small. Check the zone area data.");
                continue;
            }

            // get zone area id
            int zone_area_id_received = static_cast<int>(input_elt->Float64(0));
            if (zone_area_id_received == _ZoneArea.id)
            {
                ReportInfo(MAPSStreamedString() << "Zone ID " << zone_area_id_received << " already processed. Skipping.");
                continue;
            }

            // update zone area id
            ReportInfo(MAPSStreamedString() << "Processing zone area data for zone ID " << zone_area_id_received << "...");

            // Parse zone area polygon
            size_t current_index = 1; // start after the zone ID
            size_t zone_end_index = current_index;
            std::vector<Point2d> zone_polygon_verts;

            Point2d first_vert;
            
            while (zone_end_index + 1 < zone_area_size)
            {
                Point2d vert(input_elt->Float64(zone_end_index), input_elt->Float64(zone_end_index + 1));
                if(zone_polygon_verts.empty()){
                    first_vert = vert;
                } 
                if (!zone_polygon_verts.empty() && arePointsClose(first_vert, vert)){
                    zone_polygon_verts.push_back(vert); 
                    break;
                }
                zone_polygon_verts.push_back(vert);                
                zone_end_index += 2;
            }



            // parse static obstacles
            current_index = zone_end_index + 2; // skip the two values indicating the number of static obstacles

            std::vector<std::vector<Point2d>> static_obstacles_verts;
            while (current_index < zone_area_size)
            {
                std::vector<Point2d> curr_static_obs_verts;
                size_t obs_end_index = current_index;

                while (obs_end_index + 1 < zone_area_size)
                {
                    double x = input_elt->Float64(obs_end_index);
                    double y = input_elt->Float64(obs_end_index + 1);
                    curr_static_obs_verts.emplace_back(x, y);
                    obs_end_index += 2;

                    if ((curr_static_obs_verts.size() >= 3) &&
                        arePointsClose(curr_static_obs_verts.front(), curr_static_obs_verts.back()))
                    {
                        static_obstacles_verts.push_back(curr_static_obs_verts);
                        break; // break if we found a closed polygon
                    }
                }
                current_index = obs_end_index;
            }

            {
                std::lock_guard<std::mutex> lock(mtx);
                _ZoneArea.reset_zone_area();
                _ZoneArea.id = zone_area_id_received;
                for(auto& vert : zone_polygon_verts)
                {
                    _ZoneArea.zone_area.outer().push_back(vert);
                }
                bg::correct(_ZoneArea.zone_area);
                // Update the zone area data
                for(auto& static_obs_verts : static_obstacles_verts){
                    Polygon2d curr_sta_obs;
                    for(auto& static_obs_vert : static_obs_verts){
                        curr_sta_obs.outer().push_back(static_obs_vert);
                    }
                    bg::correct(curr_sta_obs);
                    _ZoneArea.static_obstacles.push_back(curr_sta_obs);
                }
            }

            ReportInfo(MAPSStreamedString() << "Zone area data for zone ID " << zone_area_id_received << " processed.");

            if(_PrintDebugInfo){
                std::string msg = "Zone area polygon : (";

                for(auto& verts : _ZoneArea.zone_area.outer()){
                    msg += "("  + std::to_string(bg::get<0>(verts)) + ", " + std::to_string(bg::get<1>(verts)) + "),";
                }
                msg.pop_back();
                msg += ")";
                ReportInfo(msg.c_str());
            }

            _IsZoneAreaAvailable = true;
        }
    }
}

void MAPSSGCoastPathPlanner::ReadShapesDebug()
{
    if (!DataAvailableInFIFO(Input("iShapesDebug")))
    {
        ReportInfo("Data Not Available at iShapesDebug in Coast Maneuvers");
        return;
    }

    MAPSIOElt *input_elt = StartReading(Input("iShapesDebug"));
    if (input_elt == nullptr)
    {
        ReportInfo("input is empty for iShapesDebug in Coast Maneuvers");
        return;
    }

    const int size = input_elt->VectorSize();
    if (size < 5)
    {
        ReportError("Shapes debug data too small. Check the shapes debug data.");
        return;
    }

    // parse shapes
    _DynamicObstacles.clear();
    int current_idx = 0;
    int remaining_data = size;

    while (remaining_data > 0 && current_idx < size)
    {
        int nb_points = static_cast<int>(input_elt->Float64(current_idx++));
        --remaining_data;

        if (nb_points <= 0 || remaining_data < (2 * nb_points))
        {
            break;
        }

        Polygon2d poly;
        for (int i = 0; i < nb_points; ++i)
        {
            if (current_idx + 1 >= size)
            {
                break;
            }
            double x = input_elt->Float64(current_idx++);
            double y = input_elt->Float64(current_idx++);
            remaining_data -= 2;
            poly.outer().emplace_back(x, y);
        }

        if (remaining_data > 0 && current_idx < size)
        {
            ++current_idx;
            --remaining_data;
        }

        if (poly.outer().size() >= 3)
        {
            _DynamicObstacles.push_back(poly);
        }
    }

    ReportInfo(MAPSStreamedString() << "Shapes debug data processed. " << _DynamicObstacles.size() << " shapes found.");
}

HybridAStarPlannerResult MAPSSGCoastPathPlanner::PlanPath(const Pose2d& start)
{

    auto maneuvering_zone = std::make_shared<zones::ManeuveringZone>(_ZoneArea.zone_area, "rtmaps_zone"); 
    std::vector<std::shared_ptr<zones::Zone>> zones{maneuvering_zone};
    planning::HybridAStarPlanner planner(*_Robot, zones, _PlanningBehaviors);

    planning::HybridAStarPlannerRequest request;
    request.start = start;
    request.goal = _DestinationForManeuvers.goal_pose;
    request.initial_behavior_name = GetStringProperty("planner_behavior_profile");
    request.transition_behavior_name.clear();
    request.dual_model_lut_path.clear();
    std::vector<Polygon2d> complete_local_obstacles = _ZoneArea.static_obstacles;
    complete_local_obstacles.insert(complete_local_obstacles.end(), _DynamicObstacles.begin(), _DynamicObstacles.end());
    ReportInfo(MAPSStreamedString() << "Total obstacles found : " << complete_local_obstacles.size());
    request.obstacle_polygons = complete_local_obstacles;

    return planner.plan(request);

}