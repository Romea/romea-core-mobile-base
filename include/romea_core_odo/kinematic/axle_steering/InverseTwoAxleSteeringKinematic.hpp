#ifndef romea_InverseTwoAxleSteeringKinematic_hpp
#define romea_InverseTwoAxleSteeringKinematic_hpp

//romea
#include "TwoAxleSteeringMeasure.hpp"
#include "TwoAxleSteeringKinematic.hpp"
#include "../../odometry/OdometryFrame2AS4WD.hpp"

namespace romea {

void inverseKinematic(const TwoAxleSteeringKinematic::Parameters &parameters,
                      const OdometryFrame2AS4WD & odometryFrame,
                      TwoAxleSteeringMeasure & twoAxleSteeringMeasure);


}
#endif
