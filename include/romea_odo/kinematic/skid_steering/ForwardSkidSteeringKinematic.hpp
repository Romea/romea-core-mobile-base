#ifndef romea_ForwardSkidSteeringKinematic_hpp
#define romea_ForwardSkidSteeringKinematic_hpp

//romea
#include "SkidSteeringKinematic.hpp"
#include "romea_odo/odometry/OdometryFrame2WD.hpp"
#include "romea_odo/odometry/OdometryFrame4WD.hpp"

namespace romea
{


void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const SkidSteeringCommand &commandFrame,
                      OdometryFrame2WD & odometryCommandFrame);


void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const SkidSteeringCommand &commandFrame,
                      OdometryFrame4WD & odometryCommandFrame);




}//end romea

#endif
