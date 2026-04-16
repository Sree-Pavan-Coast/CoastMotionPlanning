#include "maps_sgpathtoprovisional.h"
#include "coastmotionplanning/geometry/shape_types.hpp"

using namespace coastmotionplanning::geometry;

// ============================================================================
// RTMaps Component Definitions
// ============================================================================

MAPS_BEGIN_INPUTS_DEFINITION(MAPSSGPathToProvisional)
MAPS_INPUT("iPath", MAPS::FilterFloat64, MAPS::LastOrNextReader)
MAPS_INPUT("iZoneAreaId", MAPS::FilterInteger32, MAPS::SamplingReader)
MAPS_END_INPUTS_DEFINITION

MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSSGPathToProvisional)
MAPS_OUTPUT("oProvisionalTrajectory", MAPS::Float64, nullptr, nullptr, 10000)
MAPS_END_OUTPUTS_DEFINITION

MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSSGPathToProvisional)
MAPS_PROPERTY("plus_zone_station", 5.0, false, false)
MAPS_PROPERTY("minus_zone_station", 1.0, false, false)
MAPS_PROPERTY("corridor_width", 11.0, false, false)
MAPS_END_PROPERTIES_DEFINITION

MAPS_BEGIN_ACTIONS_DEFINITION(MAPSSGPathToProvisional)
MAPS_END_ACTIONS_DEFINITION

MAPS_COMPONENT_DEFINITION(MAPSSGPathToProvisional,
                          "MAPSSGPathToProvisional",
                          "1.0",
                          128,
                          MAPS::Threaded,
                          MAPS::Threaded,
                          -1, -1, -1, -1)

// ============================================================================
// Birth - Component Initialization
// ============================================================================

void MAPSSGPathToProvisional::Birth()
{
    CreateThread((MAPSThreadFunction)&MAPSSGPathToProvisional::ZoneAreaIDReaderThread);
}

// ============================================================================
// Core - Main Processing Loop
// ============================================================================

void MAPSSGPathToProvisional::Core()
{
    if(_IsZoneAreaAvailable)
    {
        MAPSInput &input = Input("iPath");
        MAPSIOElt *iElt = StartReading(input);
        if (iElt == nullptr)
        {
            ReportWarning("Path input is not available. Skipping output.");
            Rest(1000000);
            return;
        }

        const int path_vector_size = iElt->VectorSize();
        if (path_vector_size < 3)
        {
            ReportWarning("Path size is less than 3. Skipping output.");
            Rest(1000000);
            return;
        }

        // Parse the path and extract the coordinates
        std::vector<Point2d> path_coordinates;
        for (int i = 0; i < path_vector_size; i += 3)
        {
            const double x = iElt->Float64(i);
            const double y = iElt->Float64(i + 1);
            path_coordinates.push_back(Point2d(x, y));
        }


        MAPSIOElt *ioEltOut = StartWriting(Output("oProvisionalTrajectory"));
        
        // Fill zone/maneuver metadata
        ioEltOut->Float64(0) = static_cast<double>(_ZoneAreaID);
        ioEltOut->Float64(1) = 1.0;                                     // Speed
        ioEltOut->Float64(2) = GetFloatProperty("corridor_width");                                    // Corridor Width
        ioEltOut->Float64(3) = GetFloatProperty("minus_zone_station");                                // - Station
        ioEltOut->Float64(4) = GetFloatProperty("plus_zone_station");                                 // + Station
        ioEltOut->Float64(5) = 0.0; // Direction


        // Output the path coordinates
        long size = 6;
        for(auto coord : path_coordinates)
        {
            ioEltOut->Float64(size++) = coord.x();
            ioEltOut->Float64(size++) = coord.y();
        }
        ioEltOut->VectorSize() = size;
        ioEltOut->Timestamp() = iElt->Timestamp();
        StopWriting(ioEltOut);
        Rest(1000000);
    }
}

void MAPSSGPathToProvisional::Death()
{
    _IsZoneAreaAvailable = false;
    _ZoneAreaID = 0;
}


void MAPSSGPathToProvisional::ZoneAreaIDReaderThread()
{
    MAPSInput &input = Input("iZoneAreaId");
    while (!IsDying())
    {
        while (DataAvailableInFIFO(input))
        {
            MAPSIOElt *iElt = StartReading(input);
            if (iElt == nullptr)
            {
                break;
            }
            _ZoneAreaID = static_cast<int>(iElt->Integer32(0));
            _IsZoneAreaAvailable = true;
        }
    }
}

