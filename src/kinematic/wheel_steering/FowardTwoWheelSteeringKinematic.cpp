//romea
#include "romea_odo/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"
#include "romea_odo/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_odo/kinematic/skid_steering/SkidSteeringKinematic.hpp"

#include <iostream>

namespace romea {

//-----------------------------------------------------------------------------
void forwardKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame2FWS2FWD & commandOdometryFrame)
{
  const double halfTrack = parameters.frontTrack/2;
  const double wheelBase= parameters.rearWheelBase+parameters.frontWheelBase;
  const double & hubCarrierOffset = parameters.frontHubCarrierOffset;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & steeringAngle= commandFrame.steeringAngle;

  double tanSteeringAngle = std::tan(steeringAngle);
  double instantaneousCurvature = tanSteeringAngle/wheelBase;


  double frontLeftWheelAngle  =
      TwoWheelSteeringKinematic::computeLeftWheelAngle(tanSteeringAngle,
                                                       instantaneousCurvature,
                                                       halfTrack);

  double frontRightWheelAngle =
      TwoWheelSteeringKinematic::computeRightWheelAngle(tanSteeringAngle,
                                                        instantaneousCurvature,
                                                        halfTrack);

  double frontLeftWheelSpeed =
      OneAxleSteeringKinematic::computeLeftWheelSpeed(linearSpeed,
                                                      tanSteeringAngle,
                                                      instantaneousCurvature,
                                                      hubCarrierOffset,
                                                      halfTrack);

  double frontRightWheelSpeed =
      OneAxleSteeringKinematic::computeRightWheelSpeed(linearSpeed,
                                                       tanSteeringAngle,
                                                       instantaneousCurvature,
                                                       hubCarrierOffset,
                                                       halfTrack);

  commandOdometryFrame.frontLeftWheelSpeed = frontLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelAngle = frontLeftWheelAngle;
  commandOdometryFrame.frontRightWheelSpeed = frontRightWheelSpeed;
  commandOdometryFrame.frontRightWheelAngle = frontRightWheelAngle;
}

//-----------------------------------------------------------------------------
void forwardKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame2FWS2RWD & commandOdometryFrame)
{
  const double wheelBase= parameters.rearWheelBase + parameters.frontWheelBase;
  const double frontTrack  = parameters.frontTrack;
  const double rearTrack = parameters.rearTrack;
  const double & rearHubCarrierOffset = parameters.rearHubCarrierOffset;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & steeringAngle = commandFrame.steeringAngle;

  double tanSteeringAngle = std::tan(steeringAngle);
  double angularSpeed = tanSteeringAngle/wheelBase*linearSpeed;
  double instantaneousCurvature = tanSteeringAngle/wheelBase;


  double frontLeftWheelAngle  =
      TwoWheelSteeringKinematic::computeLeftWheelAngle(tanSteeringAngle,
                                                       instantaneousCurvature,
                                                       frontTrack/2);

  double frontRightWheelAngle =
      TwoWheelSteeringKinematic::computeRightWheelAngle(tanSteeringAngle,
                                                        instantaneousCurvature,
                                                        frontTrack/2);


  double rearLeftWheelSpeed  =
      SkidSteeringKinematic::computeLeftWheelSpeed(linearSpeed,
                                                   angularSpeed,
                                                   rearTrack+2*rearHubCarrierOffset);

  double rearRightWheelSpeed =
      SkidSteeringKinematic::computeRightWheelSpeed(linearSpeed,
                                                    angularSpeed,
                                                    rearTrack+2*rearHubCarrierOffset);

  commandOdometryFrame.rearLeftWheelSpeed=rearLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelAngle=frontLeftWheelAngle;
  commandOdometryFrame.rearRightWheelSpeed=rearRightWheelSpeed;
  commandOdometryFrame.frontRightWheelAngle=frontRightWheelAngle;
}

//-----------------------------------------------------------------------------
void forwardKinematic(const TwoWheelSteeringKinematic::Parameters & parameters,
                      const OneAxleSteeringCommand & commandFrame,
                      OdometryFrame2FWS4WD & commandOdometryFrame)
{
    const double wheelBase= parameters.rearWheelBase + parameters.frontWheelBase;
    const double frontTrack  = parameters.frontTrack;
    const double rearTrack = parameters.rearTrack;
    const double & rearHubCarrierOffset = parameters.rearHubCarrierOffset;
    const double & frontHubCarrierOffset = parameters.frontHubCarrierOffset;
    const double & linearSpeed = commandFrame.longitudinalSpeed;
    const double & steeringAngle = commandFrame.steeringAngle;

    double tanSteeringAngle = std::tan(steeringAngle);
    double angularSpeed = tanSteeringAngle/wheelBase*linearSpeed;
    double instantaneousCurvature = tanSteeringAngle/wheelBase;


    double frontLeftWheelAngle  =
        TwoWheelSteeringKinematic::computeLeftWheelAngle(tanSteeringAngle,
                                                         instantaneousCurvature,
                                                         frontTrack/2);

    double frontRightWheelAngle =
        TwoWheelSteeringKinematic::computeRightWheelAngle(tanSteeringAngle,
                                                          instantaneousCurvature,
                                                          frontTrack/2);

    double frontLeftWheelSpeed =
        OneAxleSteeringKinematic::computeLeftWheelSpeed(linearSpeed,
                                                        tanSteeringAngle,
                                                        instantaneousCurvature,
                                                        frontHubCarrierOffset,
                                                        frontTrack/2);

    double frontRightWheelSpeed =
        OneAxleSteeringKinematic::computeRightWheelSpeed(linearSpeed,
                                                         tanSteeringAngle,
                                                         instantaneousCurvature,
                                                         frontHubCarrierOffset,
                                                         frontTrack/2);

    double rearLeftWheelSpeed  =
        SkidSteeringKinematic::computeLeftWheelSpeed(linearSpeed,
                                                     angularSpeed,
                                                     rearTrack+2*rearHubCarrierOffset);

    double rearRightWheelSpeed =
        SkidSteeringKinematic::computeRightWheelSpeed(linearSpeed,
                                                      angularSpeed,
                                                      rearTrack+2*rearHubCarrierOffset);

    commandOdometryFrame.rearLeftWheelSpeed=rearLeftWheelSpeed;
    commandOdometryFrame.frontLeftWheelSpeed=frontLeftWheelSpeed;
    commandOdometryFrame.frontLeftWheelAngle=frontLeftWheelAngle;
    commandOdometryFrame.rearRightWheelSpeed=rearRightWheelSpeed;
    commandOdometryFrame.frontRightWheelSpeed=frontRightWheelSpeed;
    commandOdometryFrame.frontRightWheelAngle=frontRightWheelAngle;
}


}
