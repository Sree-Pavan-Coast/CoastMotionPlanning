#include "maps_sgmaneuverstoautonomousrerouter.h"
#include "coastmotionplanning/geometry/shape_types.hpp"
#include "coastmotionplanning/math/pose2d.hpp"
#include <vector>

using namespace coastmotionplanning::geometry;
using namespace coastmotionplanning::math;

MAPS_BEGIN_INPUTS_DEFINITION(MAPSSGManeuversToAutonomousRerouter)
MAPS_INPUT("iProvisionalTrajectoryRunningStatus", MAPS::FilterInteger32, MAPS::LastOrNextReader)
MAPS_INPUT("iTrajectoryToBeAlignedTo", MAPS::FilterFloat64, MAPS::LastOrNextReader)
MAPS_INPUT("iPositionSpeed", MAPS::FilterFloat64, MAPS::LastOrNextReader)
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
}

void MAPSSGManeuversToAutonomousRerouter::Core()
{

    MAPSInput &input = Input("iProvisionalTrajectoryRunningStatus");
    MAPSIOElt *iElt = StartReading(input);
    if (iElt == nullptr){
        ReportWarning("Provisional Trajectory mission status not available");
        Rest(1000000);
        return;
    }

    int status = iElt->Integer32();
    if(status == 0){
        Rest(1000000);
        return;
    }

    MAPSIOElt *iElt_traj = StartReading(Input("iTrajectoryToBeAlignedTo"));
    if (iElt_traj == nullptr){
        ReportWarning("Reference trajectory not available");
        Rest(1000000);
        return;
    }
    const int traj_vector_size = iElt_traj->VectorSize();
    if (traj_vector_size < 3){
        ReportWarning("Reference trajectory size is less than 3. Skipping output.");
        Rest(1000000);
        return;
    }

    // build ref traj
    std::vector<Point2d> ref_traj;
    for(int i = 0; i < traj_vector_size; i+=2){
        ref_traj.emplace_back(iElt_traj->Float64(i), iElt_traj->Float64(i+1));
    }

    bool is_reroute_success = false;
    Angle heading_threshold = Angle::from_degrees(GetFloatProperty("reroute_when_heading_threshold_deg"));
    heading_threshold.normalize();
    double dist_threshold = std::fabs(GetFloatProperty("reroute_when_distance_threshold_deg"));

    while(!is_reroute_success){
        Rest(1000000);

        MAPSIOElt *iElt_current_pos = StartReading(Input("iPositionSpeed"));
        Pose2d current(iElt_current_pos->Float64(0), iElt_current_pos->Float64(1), Angle::from_radians(iElt_current_pos->Float64(3)));



    }

        // MAPSIOElt *ioEltOut = StartWriting(Output("oProvisionalTrajectory"));
        
        // // Fill zone/maneuver metadata
        // ioEltOut->Float64(0) = static_cast<double>(_ZoneAreaID);
        // ioEltOut->Float64(1) = 1.0;                                     // Speed
        // ioEltOut->Float64(2) = GetFloatProperty("corridor_width");                                    // Corridor Width
        // ioEltOut->Float64(3) = GetFloatProperty("minus_zone_station");                                // - Station
        // ioEltOut->Float64(4) = GetFloatProperty("plus_zone_station");                                 // + Station
        // ioEltOut->Float64(5) = 0.0; // Direction


        // // Output the path coordinates
        // long size = 6;
        // for(auto coord : path_coordinates)
        // {
        //     ioEltOut->Float64(size++) = coord.x();
        //     ioEltOut->Float64(size++) = coord.y();
        // }
        // ioEltOut->VectorSize() = size;
        // ioEltOut->Timestamp() = iElt->Timestamp();
        // StopWriting(ioEltOut);
        // Rest(1000000);

}

void MAPSSGManeuversToAutonomousRerouter::Death()
{
}
