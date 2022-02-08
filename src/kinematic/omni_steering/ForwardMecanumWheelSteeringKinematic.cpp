//romea
#include "romea_core_mobile_base/kinematic/omni_steering/ForwardMecanumWheelSteeringKinematic.hpp"

//std
#include <cmath>

namespace romea {


//--------------------------------------------------------------------------
void forwardKinematic(const MecanumWheelSteeringKinematic::Parameters & parameters,
                      const OmniSteeringCommand & commandFrame,
                      OdometryFrame4WD & odometryFrame)
{
  const double & longitudinalSpeed  = commandFrame.longitudinalSpeed;
  const double & lateralSpeed  = commandFrame.longitudinalSpeed;
  const double & angularSpeed = commandFrame.angularSpeed;
  const double halfWheebase = parameters.wheelbase/2.;
  const double halfTrack = parameters.wheelTrack/2.;

  odometryFrame.frontLeftWheelSpeed=MecanumWheelSteeringKinematic::
      computeFrontLeftWheelSpeed(longitudinalSpeed,lateralSpeed,angularSpeed,halfWheebase,halfTrack);

  odometryFrame.frontRightWheelSpeed=MecanumWheelSteeringKinematic::
      computeFrontRightWheelSpeed(longitudinalSpeed,lateralSpeed,angularSpeed,halfWheebase,halfTrack);

  odometryFrame.rearLeftWheelSpeed=MecanumWheelSteeringKinematic::
      computeRearLeftWheelSpeed(longitudinalSpeed,lateralSpeed,angularSpeed,halfWheebase,halfTrack);

  odometryFrame.rearRightWheelSpeed=MecanumWheelSteeringKinematic::
      computeRearRightWheelSpeed(longitudinalSpeed,lateralSpeed,angularSpeed,halfWheebase,halfTrack);
}


}//end romea
