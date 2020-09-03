#ifndef romea_ForwardFourWheelSteeringKinematic_hpp
#define romea_ForwardFourWheelSteeringKinematic_hpp

//romea
#include "FourWheelSteeringKinematic.hpp"
#include "../../odometry/OdometryFrame4WS4WD.hpp"

namespace romea {

void forwardKinematic(const FourWheelSteeringKinematic::Parameters & kinematic,
                      const TwoAxleSteeringCommand & commandFrame,
                      OdometryFrame4WS4WD & commandOdometryFrame);


}
#endif
