// std
#include <cmath>

// romea
#include "romea_core_mobile_base/kinematic/omni_steering/ForwardMecanumWheelSteeringKinematic.hpp"


namespace romea {

//--------------------------------------------------------------------------
void forwardKinematic(const MecanumWheelSteeringKinematic::Parameters & parameters,
                      const OmniSteeringCommand & commandFrame,
                      OdometryFrame4WD &odometryFrame)
{
  const double & longitudinalSpeed  = commandFrame.longitudinalSpeed;
  const double & lateralSpeed  = commandFrame.longitudinalSpeed;
  const double & angularSpeed = commandFrame.angularSpeed;
  const double halfWheebase = parameters.wheelbase/2.;
  const double halfTrack = parameters.wheelTrack/2.;

  odometryFrame.frontLeftWheelLinearSpeed = MecanumWheelSteeringKinematic::
      computeFrontLeftWheelLinearSpeed(longitudinalSpeed,
                                 lateralSpeed,
                                 angularSpeed,
                                 halfWheebase,
                                 halfTrack);

  odometryFrame.frontRightWheelLinearSpeed = MecanumWheelSteeringKinematic::
      computeFrontRightWheelLinearSpeed(longitudinalSpeed,
                                        lateralSpeed,
                                        angularSpeed,
                                        halfWheebase,
                                        halfTrack);

  odometryFrame.rearLeftWheelLinearSpeed = MecanumWheelSteeringKinematic::
      computeRearLeftWheelLinearSpeed(longitudinalSpeed,
                                      lateralSpeed,
                                      angularSpeed,
                                      halfWheebase,
                                      halfTrack);

  odometryFrame.rearRightWheelLinearSpeed = MecanumWheelSteeringKinematic::
      computeRearRightWheelLinearSpeed(longitudinalSpeed,
                                       lateralSpeed,
                                       angularSpeed,
                                       halfWheebase,
                                       halfTrack);
}

}  // namespace romea
