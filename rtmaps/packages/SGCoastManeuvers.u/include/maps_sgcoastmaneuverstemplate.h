#pragma once

#include <maps.hpp>
#include <maps_module.h>
#include <string>

class MAPSSGCoastManeuversTemplate : public MAPSComponent
{
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSSGCoastManeuversTemplate)

private:
    bool _DebugMode = false;
};
