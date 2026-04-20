#define NOMINMAX
#include <windows.h>
#include "maps_sgheadingerrorcalc.h"
#include <string>
#include <algorithm>

#include <chrono>

using namespace coastmotionplanning::math;

MAPS_BEGIN_INPUTS_DEFINITION(MAPSSGHeadingErrorCalc)
MAPS_INPUT("iPath", MAPS::FilterFloat64, MAPS::LastOrNextReader)
MAPS_INPUT("iPositionSpeed", MAPS::FilterFloat64, MAPS::LastOrNextReader)
MAPS_INPUT("iProvisionalTrajMissionStatus", MAPS::FilterInteger32, MAPS::LastOrNextReader)
MAPS_END_INPUTS_DEFINITION

MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSSGHeadingErrorCalc)
MAPS_OUTPUT("oHeadingError", MAPS::TextAscii, nullptr, nullptr, 0)
MAPS_END_OUTPUTS_DEFINITION

MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSSGHeadingErrorCalc)
MAPS_PROPERTY_ENUM("angle_unit", "degrees|radians", 0, false, false)
MAPS_PROPERTY_ENUM("sampling_period", "500ms|1s|1.5s|2s", 0, false, false)
// MAPS_PROPERTY("stop_calc_if_vehicle_stop_for_ms", 2000000, false, false)
MAPS_END_PROPERTIES_DEFINITION

MAPS_BEGIN_ACTIONS_DEFINITION(MAPSSGHeadingErrorCalc)
MAPS_END_ACTIONS_DEFINITION

MAPS_COMPONENT_DEFINITION(MAPSSGHeadingErrorCalc,
                          "MAPSSGHeadingErrorCalc",
                          "1.0",
                          128,
                          MAPS::Threaded,
                          MAPS::Threaded,
                          -1, -1, -1, -1)

void MAPSSGHeadingErrorCalc::Birth()
{
    Output("oHeadingError").AllocOutputBuffer(8000);

    std::string set_angle_unit = GetStringProperty("angle_unit");
    if(set_angle_unit == "degrees"){
        _AngUnit = AngleUnit::Degrees;
    }else if(set_angle_unit == "radians"){
        _AngUnit = AngleUnit::Radians;
    }

    std::string set_sampling_period = GetStringProperty("sampling_period");
    if(set_sampling_period == "500ms"){
        _SamplingPeriod = 500000;
    }else if(set_sampling_period == "1s"){
        _SamplingPeriod = 1000000;
    }else if(set_sampling_period == "1.5s"){
        _SamplingPeriod = 1500000;
    }else if(set_sampling_period == "2s"){
        _SamplingPeriod = 2000000;
    }

    CreateThread((MAPSThreadFunction)&MAPSSGHeadingErrorCalc::ProvisionalTrajMissionChecker);
}

void MAPSSGHeadingErrorCalc::Core()
{

    // if _IsProvisionalTrajMissionActive, get the path data. Use it till _IsProvisionalTrajMissionActive is not active.
    if(!_IsProvisionalTrajMissionActive){

        if(_PerformOneLastHeadingCalcAfterMissionDone){
            // wait till appointment time to respect sampling period
            _Appointment += _SamplingPeriod;
            Wait(_Appointment);

            // Heading error calc and output it
            HeadingCalcAndOutput("FINAL", true);
            _PerformOneLastHeadingCalcAfterMissionDone = false;
        }

        _Path.clear();
        _Appointment = MAPS::CurrentTime();
        Rest(3000000);
    }else{
        if(_Path.empty()){
            MAPSIOElt *input = StartReading(Input("iPath"));
            if (input == nullptr) return;
            int input_size = input->VectorSize();
            for(int i = 0; i < input_size; i+=3){
                _Path.emplace_back(input->Float64(i), input->Float64(i+1), Angle::from_radians(input->Float64(i+2)));
            }
            ReportInfo("Starting HeadingErrorCalc");
        }

        // wait till appointment time to respect sampling period
        _Appointment += _SamplingPeriod;
        Wait(_Appointment);

        // Heading error calc and output it
        HeadingCalcAndOutput();
        _PerformOneLastHeadingCalcAfterMissionDone = true;
    }

}

void MAPSSGHeadingErrorCalc::Death()
{
}

void MAPSSGHeadingErrorCalc::ProvisionalTrajMissionChecker()
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
        Rest(_SamplingPeriod);
    }
    Rest(500000);
}

void MAPSSGHeadingErrorCalc::HeadingCalcAndOutput(std::string error_name, bool add_new_line_at_end_msg){
    // Get current vehicle position
    MAPSIOElt *input_elt = StartReading(Input("iPositionSpeed"));
    if (input_elt == nullptr) return;
    if (input_elt->VectorSize() <= 3){
        ReportError("Position speed input is too small. Expected x, y, speed, heading.");
        return;
    }
    Pose2d current(input_elt->Float64(0), input_elt->Float64(1), Angle::from_radians(input_elt->Float64(3)));

    // heading error 
    auto it = std::min_element(
        _Path.begin(),
        _Path.end(),
        [&current](const Pose2d& a, const Pose2d& b){
            const double dax = a.x - current.x;
            const double day = a.y - current.y;
            const double dbx = b.x - current.x;
            const double dby = b.y - current.y;

            return (dax * dax + day * day) < (dbx * dbx + dby * dby);
        }
    );

    // start calculating heading
    double heading_error = 0.0;
    std::string heading_error_msg = error_name + " " + "HEADING ERROR: ";
    if(_AngUnit == AngleUnit::Degrees){
        heading_error = ((it->theta) - (current.theta)).normalized().degrees();
        heading_error_msg += std::to_string(heading_error) + " deg";
    }else if(_AngUnit == AngleUnit::Radians){
        heading_error = ((it->theta) - (current.theta)).normalized().radians();
        heading_error_msg += std::to_string(heading_error) + " rad";
    } 
    heading_error_msg += "\n";
    if(add_new_line_at_end_msg){
        heading_error_msg += "\n";
    }

    MAPSIOElt* ioEltOut = StartWriting(Output("oHeadingError"));
    int size = (std::min)(static_cast<std::size_t>(ioEltOut->BufferSize() - 1), heading_error_msg.size());
    MAPS::Memcpy(static_cast<void *>(ioEltOut->TextAscii()),
                    static_cast<const void *>(heading_error_msg.data()),
                    size);
    ioEltOut->VectorSize() = size;
    ioEltOut->Timestamp() = input_elt->Timestamp();
    StopWriting(ioEltOut);
}