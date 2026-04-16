
#pragma once

#include <maps.hpp>
#include <maps_module.h>

class MAPSSGPathToProvisional : public MAPSComponent
{
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSSGPathToProvisional)

    bool _IsZoneAreaAvailable = false;
    int _ZoneAreaID = 0;
    void ZoneAreaIDReaderThread();

};
