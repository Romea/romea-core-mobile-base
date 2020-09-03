#ifndef romea_InverseTwoWheelSteeringKinematic_hpp
#define romea_InverseTwoWheelSteeringKinematic_hpp

//romea
#include "../KinematicMeasure.hpp"
#include "TwoWheelSteeringKinematic.hpp"
#include "../axle_steering/OneAxleSteeringMeasure.hpp"
#include "../../odometry/OdometryFrame2FWS2FWD.hpp"
#include "../../odometry/OdometryFrame2FWS2RWD.hpp"

namespace romea {

void inverseKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OdometryFrame2FWS2FWD & odometryFrame,
                      OneAxleSteeringMeasure & oneAxleSteeringMeasure);

void inverseKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OdometryFrame2FWS2RWD & odometryFrame,
                      OneAxleSteeringMeasure & oneAxleSteeringMeasure);


}

#endif
