#ifndef romea_ForwardOneAxleSteeringKinematic_hpp
#define romea_ForwardOneAxleSteeringKinematic_hpp

//romea
#include "OneAxleSteeringKinematic.hpp"
#include "../../odometry/OdometryFrame1FAS2FWD.hpp"
#include "../../odometry/OdometryFrame1FAS2RWD.hpp"

namespace romea {



void forwardKinematic(const OneAxleSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame1FAS2FWD & odometryCommandFrame);

void forwardKinematic(const OneAxleSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame1FAS2RWD & odometryCommandFrame);



}

#endif
