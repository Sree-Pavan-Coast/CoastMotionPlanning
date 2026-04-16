
#pragma once

#include <maps.hpp>
#include <maps_module.h>

#include <atomic>

enum class AngleUnit{
    Degrees,
    Radians
};

class MAPSSGHeadingErrorCalc : public MAPSComponent
{
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSSGHeadingErrorCalc)
    AngleUnit _AngUnit = AngleUnit::Degrees;

    MAPSTimestamp _Appointment;
    int _SamplingPeriod = 100000;

    // std::atomic<bool> _StartOutputHeadingErrorData = false;

    // void HeadingErrorStarter();

    // std::
};
