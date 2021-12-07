#ifndef romea_InverseFourWheelSteeringKinematic_hpp
#define romea_InverseFourWheelSteeringKinematic_hpp

//romea
#include "../KinematicMeasure.hpp"
#include "FourWheelSteeringKinematic.hpp"
#include "../axle_steering/TwoAxleSteeringMeasure.hpp"
#include "../../odometry/OdometryFrame4WS4WD.hpp"

namespace romea {


void inverseKinematic(const FourWheelSteeringKinematic::Parameters &parameters,
                      const OdometryFrame4WS4WD & odometryFrame,
                      TwoAxleSteeringMeasure & twoAxleSteeringMeasure);


}
#endif
