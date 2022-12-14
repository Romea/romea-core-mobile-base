// romea
#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"

namespace romea {

//--------------------------------------------------------------------------
void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const SkidSteeringCommand &commandFrame,
                      OdometryFrame2TD & odometryCommandFrame)
{
  OdometryFrame2WD odometryCommandFrame2WD;
  forwardKinematic(parameters, commandFrame, odometryCommandFrame2WD);
  odometryCommandFrame.leftTrackLinearSpeed = odometryCommandFrame2WD.leftWheelLinearSpeed;
  odometryCommandFrame.rightTrackLinearSpeed =  odometryCommandFrame2WD.rightWheelLinearSpeed;
}

//--------------------------------------------------------------------------
void forwardKinematic(const SkidSteeringKinematic::Parameters &parameters,
                      const SkidSteeringCommand &commandFrame,
                      OdometryFrame2WD & odometryCommandFrame)
{
  const double track = parameters.wheelTrack;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & angularSpeed = commandFrame.angularSpeed;

  odometryCommandFrame.leftWheelLinearSpeed = SkidSteeringKinematic::
      computeLeftWheelLinearSpeed(linearSpeed, angularSpeed, track);
  odometryCommandFrame.rightWheelLinearSpeed = SkidSteeringKinematic::
      computeRightWheelLinearSpeed(linearSpeed, angularSpeed, track);
}

//--------------------------------------------------------------------------
void forwardKinematic(const SkidSteeringKinematic::Parameters & parameters,
                      const SkidSteeringCommand & commandFrame,
                      OdometryFrame4WD &odometryCommandFrame)
{
  const double track = parameters.wheelTrack;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & angularSpeed = commandFrame.angularSpeed;

  double leftWheelSpeed = SkidSteeringKinematic::
      computeLeftWheelLinearSpeed(linearSpeed, angularSpeed, track);
  double rightWheelSpeed = SkidSteeringKinematic::
      computeRightWheelLinearSpeed(linearSpeed, angularSpeed, track);

  odometryCommandFrame.frontLeftWheelLinearSpeed = leftWheelSpeed;
  odometryCommandFrame.frontRightWheelLinearSpeed = rightWheelSpeed;
  odometryCommandFrame.rearLeftWheelLinearSpeed = leftWheelSpeed;
  odometryCommandFrame.rearRightWheelLinearSpeed = rightWheelSpeed;
}
 
}  // namespace romea
