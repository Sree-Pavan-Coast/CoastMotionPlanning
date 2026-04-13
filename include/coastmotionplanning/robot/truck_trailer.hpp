#pragma once

#include "coastmotionplanning/robot/robot_base.hpp"

namespace coastmotionplanning {
namespace robot {

struct TractorParams {
    double width;
    double wheelbase;
    double front_overhang;
    double rear_overhang;
    double hitch_offset; // distance from tractor rear axle to hitch point (positive backwards)
};

struct TrailerParams {
    double width;
    double wheelbase; // distance from hitch to trailer axle
    double front_overhang; // distance the trailer body extends forward from the hitch
    double rear_overhang; // distance from trailer front/axle to rear. Wait, let's standardize:
};

// Let's refine TrailerParams:
// hitch to axle = wheelbase
// axle to back = rear_overhang
// axle to front = wheelbase - front_overhang? No, hitch to front = front_overhang. So front most point from axle is wheelbase + front_overhang.

class TruckTrailer : public RobotBase {
public:
    struct Params {
        double tractor_width{0.0};
        double tractor_wheelbase{0.0};
        double tractor_front_overhang{0.0};
        double tractor_rear_overhang{0.0};
        double hitch_offset{0.0}; // from tractor rear axle backward

        double trailer_width{0.0};
        double trailer_wheelbase{0.0}; // from hitch backwards to trailer axle
        double trailer_front_overhang{0.0}; // from hitch backwards to trailer front face (if 0, front of trailer is at hitch, if <0 trailer extends over tractor)
        double trailer_rear_overhang{0.0}; // from trailer axle backwards to trailer rear face
    };

    explicit TruckTrailer(const Params& params);
    ~TruckTrailer() override = default;

    geometry::Polygon2d getRobotFootprint(const RobotState& state) const override;
    
    // Specific accessors
    const Params& getParams() const { return params_; }

private:
    Params params_;
};

} // namespace robot
} // namespace coastmotionplanning
