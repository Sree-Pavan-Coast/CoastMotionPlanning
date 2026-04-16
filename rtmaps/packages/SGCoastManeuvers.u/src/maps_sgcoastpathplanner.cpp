#include "maps_sgcoastpathplanner.h"

#include "coastmotionplanning/geometry/shape_types.hpp"

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
MAPS_PROPERTY_SUBTYPE("robot_params_file_path", "", false, true,
                      MAPS::PropertySubTypeFile | MAPS::PropertySubTypeMustExist)
MAPS_PROPERTY_SUBTYPE("planning_params_file_path", "", false, true,
                      MAPS::PropertySubTypeFile | MAPS::PropertySubTypeMustExist)
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
    CreateThread((MAPSThreadFunction)&MAPSSGCoastPathPlanner::ZoneAreaReaderThread);

    // [DEBUG]check if coast motion planning library is working correctly
    coast::Point2d point(0.0, 0.0);
    coast::Polygon polygon;
    polygon.addVertex(point);
    ReportInfo(MAPSStreamedString() << "Coast motion planning library is working correctly. Polygon: " << polygon.toString());
    ReportInfo(MAPSStreamedString() << "Polygon vertices: " << polygon.numVertices());
    ReportInfo(MAPSStreamedString() << "Polygon area: " << polygon.area());
    ReportInfo(MAPSStreamedString() << "Polygon perimeter: " << polygon.perimeter());
    ReportInfo(MAPSStreamedString() << "Polygon centroid: " << polygon.centroid().x << ", " << polygon.centroid().y);
    ReportInfo(MAPSStreamedString() << "Polygon bounding box: " << polygon.boundingBox().x1 << ", " << polygon.boundingBox().y1 << " - " << polygon.boundingBox().x2 << ", " << polygon.boundingBox().y2);
    ReportInfo(MAPSStreamedString() << "Polygon convex hull: " << polygon.convexHull().numVertices());
    ReportInfo(MAPSStreamedString() << "Polygon convex hull vertices: " << polygon.convexHull().vertices[0].x << ", " << polygon.convexHull().vertices[0].y);
}

void MAPSSGCoastPathPlanner::Core()
{

    // Check if zone area is available.
    if (_ZoneArea.IsInit())
    {
        MAPSIOElt *input_elt = StartReading(Input("iDestinationForManeuvers"));
        if (input_elt == nullptr)
            return;

        // Parse the destination for maneuvers
        double x = input_elt->Float64(0);
        double y = input_elt->Float64(1);
        double heading_rad = input_elt->Float64(2);
        _DestinationForManeuvers.pose.setX(x);
        _DestinationForManeuvers.pose.setY(y);
        _DestinationForManeuvers.pose.setHeadingRad(heading_rad);

        // Parse the station polygon
        _DestinationForManeuvers.goal_station.clear();
        for (int i = 3; i < input_elt->VectorSize(); i += 2)
        {
            _DestinationForManeuvers.goal_station.addVertex(input_elt->Float64(i), input_elt->Float64(i + 1));
        }

        // Parse starting position
        input_elt = StartReading(Input("iPositionSpeed"));
        if (input_elt == nullptr)
            return;
        if (input_elt->VectorSize() <= 3)
        {
            ReportError("Position speed input is too small. Expected x, y, speed, heading.");
            return;
        }
        Pose start;
        start.setX(input_elt->Float64(0));
        start.setY(input_elt->Float64(1));
        start.setHeadingRad(input_elt->Float64(3));

        // Parse shapes for debug
        ReadShapesDebug();

        // Plan the path
        const std::vector<Pose> path = PlanPath(start);

        MAPSIOElt *ioEltOut = StartWriting(Output("oPath"));
        long size = 0;
        constexpr long kMaxOutputValues = 10000;
        for (const Pose &point : path)
        {
            if (size + 3 > kMaxOutputValues)
            {
                break;
            }

            ioEltOut->Float64(size++) = point.x;
            ioEltOut->Float64(size++) = point.y;
            ioEltOut->Float64(size++) = point.heading_rad;
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
            coast::Polygon zone_polygon;
            while (zone_end_index + 1 < zone_area_size)
            {
                coast::Point2d point;
                point.x = input_elt->Float64(zone_end_index);
                point.y = input_elt->Float64(zone_end_index + 1);
                zone_polygon.addVertex(point);

                zone_end_index += 2;
                if (zone_polygon.numVertices() >= 3 && (zone_polygon.distFirstVertToLastVert() < 1e-6))
                {
                    break;
                }
            }

            // parse static obstacles
            current_index = zone_end_index + 2; // skip the two values indicating the number of static obstacles

            Obstacles static_obstacles;
            while (current_index < zone_area_size)
            {
                coast::Polygon curr_static_obs;
                size_t obs_end_index = current_index;

                while (obs_end_index + 1 < zone_area_size)
                {
                    double x = input_elt->Float64(obs_end_index);
                    double y = input_elt->Float64(obs_end_index + 1);
                    coast::Point2d point(x, y);

                    curr_static_obs.addVertex(point);
                    obs_end_index += 2;

                    if (curr_static_obs.numVertices() >= 3 &&
                        (curr_static_obs.distFirstVertToLastVert() < 1e-6))
                    {
                        static_obstacles.push_back(curr_static_obs);
                        break; // break if we found a closed polygon
                    }
                }
                current_index = obs_end_index;
            }

            // Update the zone area data
            {
                std::lock_guard<std::mutex> lock(mtx);
                _ZoneArea.id = zone_area_id_received;
                _ZoneArea.polygon = std::move(zone_polygon);
                _ZoneArea.static_obstacles = std::move(static_obstacles);
            }
            ReportInfo(MAPSStreamedString() << "Zone area data for zone ID " << zone_area_id_received << " processed.");
        }
    }
}

void MAPSSGCoastPathPlanner::ReadShapesDebug()
{
    if (!DataAvailableInFIFO(Input("iShapesDebug")))
    {
        return;
    }

    MAPSIOElt *input_elt = StartReading(Input("iShapesDebug"));
    if (input_elt == nullptr)
    {
        return;
    }

    const int size = input_elt->VectorSize();
    if (size < 5)
    {
        ReportError("Shapes debug data too small. Check the shapes debug data.");
        return;
    }

    // parse shapes
    _Shapes.clear();
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

        coast::Polygon poly;
        for (int i = 0; i < nb_points; ++i)
        {
            if (current_idx + 1 >= size)
            {
                break;
            }
            double x = input_elt->Float64(current_idx++);
            double y = input_elt->Float64(current_idx++);
            remaining_data -= 2;
            poly.addVertex(x, y);
        }

        if (remaining_data > 0 && current_idx < size)
        {
            ++current_idx;
            --remaining_data;
        }

        if (poly.numVertices() >= 3)
        {
            _Shapes.push_back(poly);
        }
    }

    ReportInfo(MAPSStreamedString() << "Shapes debug data processed. " << _Shapes.size() << " shapes found.");
}

// std::vector<Pose> MAPSSGCoastPathPlanner::PlanPath() const
// {

//     return path;
// }
