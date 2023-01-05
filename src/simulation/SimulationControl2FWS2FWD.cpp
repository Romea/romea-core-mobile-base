// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_mobile_base/simulation/SimulationControl2FWS2FWD.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SimulationCommand2FWS2FWD toSimulationCommand2FWS2FWD(
  const HardwareCommand2FWS2FWD & hardwareCommand,
  const double & rearLeftWheelSpinningSetPoint,
  const double & rearRightWheelSpinningSetPoint)
{
  return {hardwareCommand.frontLeftWheelSteeringAngle,
    hardwareCommand.frontRightWheelSteeringAngle,
    hardwareCommand.frontLeftWheelSpinningSetPoint,
    hardwareCommand.frontRightWheelSpinningSetPoint,
    rearLeftWheelSpinningSetPoint,
    rearRightWheelSpinningSetPoint};
}

//-----------------------------------------------------------------------------
SimulationCommand2FWS2FWD toSimulationCommand2FWS2FWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & rearTrack,
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const double & frontHubCarrierOffset,
  const double & rearHubCarrierOffset,
  const HardwareCommand2FWS2FWD & hardwareCommand)
{
  double fullRearTrack = rearTrack + 2 * rearHubCarrierOffset;

  const double & frontLeftWheelLinearSpeed =
    hardwareCommand.frontLeftWheelSpinningSetPoint * frontWheelRadius;
  const double & frontRightWheelLinearSpeed =
    hardwareCommand.frontRightWheelSpinningSetPoint * frontWheelRadius;
  const double & frontLeftWheelSteeringAngle =
    hardwareCommand.frontLeftWheelSteeringAngle;
  const double & frontRightWheelSteeringAngle =
    hardwareCommand.frontRightWheelSteeringAngle;

  double instantaneousCurvature = TwoWheelSteeringKinematic::
    computeInstantaneousCurvature(
    frontLeftWheelSteeringAngle,
    frontRightWheelSteeringAngle,
    wheelbase, frontTrack);

  double linearSpeed = 0.5 * (frontLeftWheelLinearSpeed / TwoWheelSteeringKinematic::
    computeWheelLinearSpeedRatio(
      -instantaneousCurvature * wheelbase,
      -instantaneousCurvature,
      frontHubCarrierOffset,
      frontTrack / 2.) +
    frontRightWheelLinearSpeed / TwoWheelSteeringKinematic::
    computeWheelLinearSpeedRatio(
      instantaneousCurvature * wheelbase,
      instantaneousCurvature,
      frontHubCarrierOffset,
      frontTrack / 2.));

  double angularSpeed = instantaneousCurvature * linearSpeed;

  double rearLeftWheelLinearSpeed = SkidSteeringKinematic::
    computeLeftWheelLinearSpeed(linearSpeed, angularSpeed, fullRearTrack);

  double rearRightWheelLinearSpeed = SkidSteeringKinematic::
    computeRightWheelLinearSpeed(linearSpeed, angularSpeed, fullRearTrack);

  return toSimulationCommand2FWS2FWD(
    hardwareCommand,
    rearLeftWheelLinearSpeed / rearWheelRadius,
    rearRightWheelLinearSpeed / rearWheelRadius);
}


//-----------------------------------------------------------------------------
HardwareState2FWS2FWD toHardwareState2FWS2FWD(const SimulationState2FWS2FWD & simulationState)
{
  return {simulationState.frontLeftWheelSteeringAngle,
    simulationState.frontRightWheelSteeringAngle,
    simulationState.frontLeftWheelSpinningMotion,
    simulationState.frontRightWheelSpinningMotion};
}

}  // namespace romea
