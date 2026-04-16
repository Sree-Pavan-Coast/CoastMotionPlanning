#include "maps_sgcoastmaneuverstemplate.h"

#include <sstream>
#include <string>

MAPS_BEGIN_INPUTS_DEFINITION(MAPSSGCoastManeuversTemplate)
    MAPS_INPUT("iInput", MAPS::FilterFloat64, MAPS::LastOrNextReader)
MAPS_END_INPUTS_DEFINITION

MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSSGCoastManeuversTemplate)
    MAPS_OUTPUT("oOutput", MAPS::Float64, nullptr, nullptr, 1)
MAPS_END_OUTPUTS_DEFINITION

MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSSGCoastManeuversTemplate)
    MAPS_PROPERTY("debug_mode", false, false, false)
MAPS_END_PROPERTIES_DEFINITION

MAPS_BEGIN_ACTIONS_DEFINITION(MAPSSGCoastManeuversTemplate)
MAPS_END_ACTIONS_DEFINITION

MAPS_COMPONENT_DEFINITION(MAPSSGCoastManeuversTemplate,
                          "MAPSSGCoastManeuversTemplate",
                          "1.0",
                          128,
                          MAPS::Threaded,
                          MAPS::Threaded,
                          -1,
                          -1,
                          -1,
                          -1)

void MAPSSGCoastManeuversTemplate::Birth()
{
}

void MAPSSGCoastManeuversTemplate::Core()
{
}

void MAPSSGCoastManeuversTemplate::Death()
{
}
