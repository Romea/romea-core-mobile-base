#ifndef romea_ForwardTwoWheelSteeringKinematic_hpp
#define romea_ForwardTwoWheelSteeringKinematic_hpp

//romea
#include "TwoWheelSteeringKinematic.hpp"
#include "../../odometry/OdometryFrame2FWS2FWD.hpp"
#include "../../odometry/OdometryFrame2FWS2RWD.hpp"

namespace romea {

void forwardKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame2FWS2FWD & commandOdometryFrame);

void forwardKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame2FWS2RWD & commandOdometryFrame);


}

#endif
