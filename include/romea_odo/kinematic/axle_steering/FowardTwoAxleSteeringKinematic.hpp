#ifndef romea_ForwardTwoAxleSteeringKinematic_hpp
#define romea_ForwardTwoAxleSteeringKinematic_hpp

//romea
#include "TwoAxleSteeringKinematic.hpp"
#include "../../odometry/OdometryFrame2AS4WD.hpp"

namespace romea {

void forwardKinematic(const TwoAxleSteeringKinematic::Parameters & parameters,
                      const TwoAxleSteeringCommand &commandFrame,
                      OdometryFrame2AS4WD & odometryCommandFrame);



}
#endif
