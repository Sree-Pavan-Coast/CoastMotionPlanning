
#pragma once

#include <maps.hpp>
#include <maps_module.h>
#include <atomic>
#include <vector>
#include <string>
#include "coastmotionplanning/math/pose2d.hpp"

using namespace coastmotionplanning::math;

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
    std::atomic<bool> _IsProvisionalTrajMissionActive = false;
    std::vector<Pose2d> _Path;
    bool _PerformOneLastHeadingCalcAfterMissionDone = false;

    void ProvisionalTrajMissionChecker();

    void HeadingCalcAndOutput(std::string error_name = "", bool add_new_line_at_end_msg = false);
};
