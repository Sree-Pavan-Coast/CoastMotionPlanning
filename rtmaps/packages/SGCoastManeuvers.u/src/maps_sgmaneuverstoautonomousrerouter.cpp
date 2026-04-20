#include "maps_sgmaneuverstoautonomousrerouter.h"
#include "coastmotionplanning/geometry/shape_types.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include <vector>

using namespace coastmotionplanning::geometry;
using namespace coastmotionplanning::math;

MAPS_BEGIN_INPUTS_DEFINITION(MAPSSGManeuversToAutonomousRerouter)
MAPS_INPUT("iProvisionalTrajMissionStatus", MAPS::FilterInteger32, MAPS::LastOrNextReader)
MAPS_INPUT("iRefTrajForReRoute", MAPS::FilterFloat64, MAPS::LastOrNextReader)
MAPS_INPUT("iPositionSpeed", MAPS::FilterFloat64, MAPS::LastOrNextReader)
MAPS_INPUT("iReRouteStatus", MAPS::FilterInteger32, MAPS::LastOrNextReader)
MAPS_END_INPUTS_DEFINITION

MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSSGManeuversToAutonomousRerouter)
MAPS_OUTPUT("oReroute", MAPS::Integer32, nullptr, nullptr, 1)
MAPS_END_OUTPUTS_DEFINITION

MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSSGManeuversToAutonomousRerouter)
MAPS_PROPERTY("reroute_when_heading_threshold_deg", 5.0, false, false)
MAPS_PROPERTY("reroute_when_distance_threshold_deg", 2.0, false, false)
// MAPS_PROPERTY("corridor_width", 11.0, false, false)
MAPS_END_PROPERTIES_DEFINITION

MAPS_BEGIN_ACTIONS_DEFINITION(MAPSSGManeuversToAutonomousRerouter)
MAPS_END_ACTIONS_DEFINITION

MAPS_COMPONENT_DEFINITION(MAPSSGManeuversToAutonomousRerouter,
                          "MAPSSGManeuversToAutonomousRerouter",
                          "1.0",
                          128,
                          MAPS::Threaded,
                          MAPS::Threaded,
                          -1, -1, -1, -1)

void MAPSSGManeuversToAutonomousRerouter::Birth()
{
    _HeadingThreshold = Angle::from_degrees(GetFloatProperty("reroute_when_heading_threshold_deg"));
    _DistThreshold = std::fabs(GetFloatProperty("reroute_when_distance_threshold_deg"));

    CreateThread((MAPSThreadFunction)&MAPSSGManeuversToAutonomousRerouter::ProvisionalTrajMissionChecker);
    CreateThread((MAPSThreadFunction)&MAPSSGManeuversToAutonomousRerouter::ReRouteMissionChecker);
}

void MAPSSGManeuversToAutonomousRerouter::Core()
{
    if(!_IsProvisionalTrajMissionActive){
        _RefPathCoords.clear();
        Rest(2000000); // 2sec
    }else{
        if(_RefPathCoords.empty()){
            MAPSIOElt *input = StartReading(Input("iRefTrajForReRoute"));
            if (input == nullptr) return;
            int input_size = input->VectorSize();
            for(int i = 0; i < input_size; i+=2){
                _RefPathCoords.emplace_back(input->Float64(i), input->Float64(i+1));
            }
            ReportInfo("Starting Maneuver to Autonomous reRoute");
        }
        Rest(500000); // 200ms

        // Get current vehicle position
        MAPSIOElt *input_elt = StartReading(Input("iPositionSpeed"));
        if (input_elt == nullptr) return;
        if (input_elt->VectorSize() <= 3){
            ReportError("Position speed input is too small. Expected x, y, speed, heading.");
            return;
        }
        Pose2d current(input_elt->Float64(0), input_elt->Float64(1), Angle::from_radians(input_elt->Float64(3)));

        // [TO DO] write logic to check the current pose against the reference path based on distance and if
        // threshold is below then check for heading error between the reference path and the pose heading. We do nto worry about the path's direction for this check. 
        // We just get the least angle difference(convert to positive) against the path. then we check if below heading threshold.

        bool trigger_reroute = false;

        // then trigger reroute
        if(!_IsReRouteMissionSuccess){
            if(trigger_reroute){
                MAPSIOElt *ioEltOut = StartWriting(Output("oReroute"));
                ioEltOut->Integer32() = 1;
                ioEltOut->Timestamp() = input_elt->Timestamp();
                StopWriting(ioEltOut);
            }
        }else{
            _IsProvisionalTrajMissionActive.store(false);
            _RefPathCoords.clear();
        }
    }
    Rest(1000000); // 1sec
}

void MAPSSGManeuversToAutonomousRerouter::Death()
{
}

void MAPSSGManeuversToAutonomousRerouter::ProvisionalTrajMissionChecker()
{
    auto &input = Input("iProvisionalTrajMissionStatus");
    while (!IsDying()){
        if (DataAvailableInFIFO(input)){
            MAPSIOElt *input_elt = StartReading(input);
            if (input_elt == nullptr){
                Rest(2000000);
                continue;
            }
            int curr_prov_traj_mission_status = input_elt->Integer32();
            if(curr_prov_traj_mission_status == 1){
                if(!_IsProvisionalTrajMissionActive.load()){
                    _IsProvisionalTrajMissionActive.store(true);
                }
            }else if(curr_prov_traj_mission_status == 0){
                if(_IsProvisionalTrajMissionActive.load()){
                    _IsProvisionalTrajMissionActive.store(false);
                }
            }
        }
        Rest(2000000);
    }
    Rest(500000);
}

void MAPSSGManeuversToAutonomousRerouter::ReRouteMissionChecker()
{
    auto &input = Input("iReRouteStatus");
    while (!IsDying()){
        if (DataAvailableInFIFO(input)){
            MAPSIOElt *input_elt = StartReading(input);
            if (input_elt == nullptr){
                Rest(2000000);
                continue;
            }
            int curr_reroute_mission_status = input_elt->Integer32();
            if(curr_reroute_mission_status == 1){
                if(!_IsReRouteMissionSuccess.load()){
                    _IsReRouteMissionSuccess.store(true);
                }
            }else if(curr_reroute_mission_status == 0){
                if(_IsReRouteMissionSuccess.load()){
                    _IsReRouteMissionSuccess.store(false);
                }
            }
        }
        Rest(2000000);
    }
    Rest(500000);
}
