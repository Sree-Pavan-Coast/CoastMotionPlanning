
#pragma once

#include <maps.hpp>
#include <maps_module.h>
#include <atomic>

class MAPSSGPathToProvisional : public MAPSComponent
{
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSSGPathToProvisional)

    bool _IsZoneAreaAvailable = false;
    std::atomic<int> _ZoneAreaID = 0;
    void ZoneAreaIDReaderThread();

};
