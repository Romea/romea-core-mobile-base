//romea
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"

#include <iostream>

namespace romea {

//-----------------------------------------------------------------------------
void forwardKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame2FWS2FWD & commandOdometryFrame)
{
  const double halfTrack = parameters.frontWheelTrack/2;
  const double wheelBase= parameters.rearWheelBase+parameters.frontWheelBase;
  const double & hubCarrierOffset = parameters.frontHubCarrierOffset;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & steeringAngle= commandFrame.steeringAngle;

  double tanSteeringAngle = std::tan(steeringAngle);
  double instantaneousCurvature = tanSteeringAngle/wheelBase;


  double frontLeftWheelAngle  = TwoWheelSteeringKinematic::
      computeLeftWheelSteeringAngle(tanSteeringAngle,
                                    instantaneousCurvature,
                                    halfTrack);

  double frontRightWheelAngle = TwoWheelSteeringKinematic::
      computeRightWheelSteeringAngle(tanSteeringAngle,
                                     instantaneousCurvature,
                                     halfTrack);

  double frontLeftWheelSpeed = OneAxleSteeringKinematic::
      computeLeftWheelLinearSpeed(linearSpeed,
                                  tanSteeringAngle,
                                  instantaneousCurvature,
                                  hubCarrierOffset,
                                  halfTrack);

  double frontRightWheelSpeed = OneAxleSteeringKinematic::
      computeRightWheelLinearSpeed(linearSpeed,
                                   tanSteeringAngle,
                                   instantaneousCurvature,
                                   hubCarrierOffset,
                                   halfTrack);

  commandOdometryFrame.frontLeftWheelLinearSpeed = frontLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelSteeringAngle = frontLeftWheelAngle;
  commandOdometryFrame.frontRightWheelLinearSpeed = frontRightWheelSpeed;
  commandOdometryFrame.frontRightWheelSteeringAngle = frontRightWheelAngle;
}

//-----------------------------------------------------------------------------
void forwardKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame2FWS2RWD & commandOdometryFrame)
{
  const double wheelBase= parameters.rearWheelBase + parameters.frontWheelBase;
  const double frontTrack  = parameters.frontWheelTrack;
  const double rearTrack = parameters.rearWheelTrack;
  const double & rearHubCarrierOffset = parameters.rearHubCarrierOffset;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & steeringAngle = commandFrame.steeringAngle;

  double tanSteeringAngle = std::tan(steeringAngle);
  double angularSpeed = tanSteeringAngle/wheelBase*linearSpeed;
  double instantaneousCurvature = tanSteeringAngle/wheelBase;


  double frontLeftWheelAngle  = TwoWheelSteeringKinematic::
      computeLeftWheelSteeringAngle(tanSteeringAngle,
                                    instantaneousCurvature,
                                    frontTrack/2);

  double frontRightWheelAngle = TwoWheelSteeringKinematic::
      computeRightWheelSteeringAngle(tanSteeringAngle,
                                     instantaneousCurvature,
                                     frontTrack/2);

  double rearLeftWheelSpeed  = SkidSteeringKinematic::
      computeLeftWheelLinearSpeed(linearSpeed,
                                  angularSpeed,
                                  rearTrack+2*rearHubCarrierOffset);

  double rearRightWheelSpeed = SkidSteeringKinematic::
      computeRightWheelLinearSpeed(linearSpeed,
                                   angularSpeed,
                                   rearTrack+2*rearHubCarrierOffset);

  commandOdometryFrame.rearLeftWheelLinearSpeed=rearLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelSteeringAngle=frontLeftWheelAngle;
  commandOdometryFrame.rearRightWheelLinearSpeed=rearRightWheelSpeed;
  commandOdometryFrame.frontRightWheelSteeringAngle=frontRightWheelAngle;
}

//-----------------------------------------------------------------------------
void forwardKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame2FWS4WD & commandOdometryFrame)
{
  const double wheelBase= parameters.rearWheelBase + parameters.frontWheelBase;
  const double frontTrack  = parameters.frontWheelTrack;
  const double rearTrack = parameters.rearWheelTrack;
  const double & rearHubCarrierOffset = parameters.rearHubCarrierOffset;
  const double & frontHubCarrierOffset = parameters.frontHubCarrierOffset;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & steeringAngle = commandFrame.steeringAngle;

  double tanSteeringAngle = std::tan(steeringAngle);
  double angularSpeed = tanSteeringAngle/wheelBase*linearSpeed;
  double instantaneousCurvature = tanSteeringAngle/wheelBase;


  double frontLeftWheelAngle  = TwoWheelSteeringKinematic::
      computeLeftWheelSteeringAngle(tanSteeringAngle,
                                    instantaneousCurvature,
                                    frontTrack/2);

  double frontRightWheelAngle =TwoWheelSteeringKinematic::
      computeRightWheelSteeringAngle(tanSteeringAngle,
                                     instantaneousCurvature,
                                     frontTrack/2);

  double frontLeftWheelSpeed = OneAxleSteeringKinematic::
      computeLeftWheelLinearSpeed(linearSpeed,
                                  tanSteeringAngle,
                                  instantaneousCurvature,
                                  frontHubCarrierOffset,
                                  frontTrack/2);

  double frontRightWheelSpeed = OneAxleSteeringKinematic::
      computeRightWheelLinearSpeed(linearSpeed,
                                   tanSteeringAngle,
                                   instantaneousCurvature,
                                   frontHubCarrierOffset,
                                   frontTrack/2);

  double rearLeftWheelSpeed = SkidSteeringKinematic::
      computeLeftWheelLinearSpeed(linearSpeed,
                                  angularSpeed,
                                  rearTrack+2*rearHubCarrierOffset);

  double rearRightWheelSpeed = SkidSteeringKinematic::
      computeRightWheelLinearSpeed(linearSpeed,
                                   angularSpeed,
                                   rearTrack+2*rearHubCarrierOffset);

  commandOdometryFrame.rearLeftWheelLinearSpeed=rearLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelLinearSpeed=frontLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelSteeringAngle=frontLeftWheelAngle;
  commandOdometryFrame.rearRightWheelLinearSpeed=rearRightWheelSpeed;
  commandOdometryFrame.frontRightWheelLinearSpeed=frontRightWheelSpeed;
  commandOdometryFrame.frontRightWheelSteeringAngle=frontRightWheelAngle;
}


}
