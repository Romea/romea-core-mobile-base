// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_mobile_base/simulation/SimulationControl1FAS2FWD.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SimulationCommand1FAS2FWD toSimulationCommand1FAS2FWD(
  const HardwareCommand1FAS2FWD & hardwareCommand,
  const double & frontLeftWheelSteeringAngleCommand,
  const double & frontRightWheelSteeringAngleCommand,
  const double & rearLeftWheelAngularSpeedCommand,
  const double & rearRightWheelAngularSpeedCommand)
{

  return {hardwareCommand.frontAxleSteeringAngle,
    frontLeftWheelSteeringAngleCommand,
    frontRightWheelSteeringAngleCommand,
    hardwareCommand.frontLeftWheelSpinningSetPoint,
    hardwareCommand.frontRightWheelSpinningSetPoint,
    rearLeftWheelAngularSpeedCommand,
    rearRightWheelAngularSpeedCommand};
}

//-----------------------------------------------------------------------------
SimulationCommand1FAS2FWD toSimulationCommand1FAS2FWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & rearTrack,
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const double & frontHubCarrierOffset,
  const double & rearHubCarrierOffset,
  const HardwareCommand1FAS2FWD & hardwareCommand)
{
  double fullRearTrack = rearTrack + 2 * rearHubCarrierOffset;

  const double & frontLeftWheelLinearSpeed =
    hardwareCommand.frontLeftWheelSpinningSetPoint * frontWheelRadius;

  const double & frontRightWheelLinearSpeed =
    hardwareCommand.frontRightWheelSpinningSetPoint * frontWheelRadius;

  const double & tanAxleSteeringAngle =
    std::tan(hardwareCommand.frontAxleSteeringAngle);

  double instantaneousCurvature = OneAxleSteeringKinematic::
    computeInstantaneousCurvature(tanAxleSteeringAngle, wheelbase);


  double frontLeftWheelSteeringAngle = TwoWheelSteeringKinematic::
    computeLeftWheelSteeringAngle(
    tanAxleSteeringAngle,
    instantaneousCurvature,
    frontTrack / 2.);

  double frontRightWheelSteeringAngle = TwoWheelSteeringKinematic::
    computeRightWheelSteeringAngle(
    tanAxleSteeringAngle,
    instantaneousCurvature,
    frontTrack / 2.);


  double linearSpeed = 0.5 * (frontLeftWheelLinearSpeed / TwoWheelSteeringKinematic::
    computeWheelLinearSpeedRatio(
      -tanAxleSteeringAngle,
      -instantaneousCurvature,
      frontHubCarrierOffset,
      frontTrack / 2.) +
    frontRightWheelLinearSpeed / TwoWheelSteeringKinematic::
    computeWheelLinearSpeedRatio(
      tanAxleSteeringAngle,
      instantaneousCurvature,
      frontHubCarrierOffset,
      frontTrack / 2.));

  double angularSpeed = instantaneousCurvature * linearSpeed;

  double rearLeftWheelLinearSpeed = SkidSteeringKinematic::
    computeLeftWheelLinearSpeed(linearSpeed, angularSpeed, fullRearTrack);

  double rearRightWheelLinearSpeed = SkidSteeringKinematic::
    computeRightWheelLinearSpeed(linearSpeed, angularSpeed, fullRearTrack);

  return toSimulationCommand1FAS2FWD(
    hardwareCommand,
    frontLeftWheelSteeringAngle,
    frontRightWheelSteeringAngle,
    rearLeftWheelLinearSpeed / rearWheelRadius,
    rearRightWheelLinearSpeed / rearWheelRadius);
}

//-----------------------------------------------------------------------------
HardwareState1FAS2FWD toHardwareState1FAS2FWD(
  const SimulationState1FAS2FWD & simulationState,
  const double frontAxleSteeringAngle)
{
  return {frontAxleSteeringAngle,
    simulationState.frontLeftWheelSpinningMotion,
    simulationState.frontRightWheelSpinningMotion};
}

//-----------------------------------------------------------------------------
HardwareState1FAS2FWD toHardwareState1FAS2FWD(
  const double & wheelbase,
  const double & frontTrack,
  const SimulationState1FAS2FWD & simulationState)
{
  double frontAxleSteeringAngle = TwoWheelSteeringKinematic::
    computeSteeringAngle(
    simulationState.frontLeftWheelSteeringAngle,
    simulationState.frontRightWheelSteeringAngle,
    wheelbase, frontTrack);

  return toHardwareState1FAS2FWD(simulationState, frontAxleSteeringAngle);
}

}  // namespace romea
