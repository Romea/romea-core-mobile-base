// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "romea_core_mobile_base/simulation/SimulationControl2FWS2FWD.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea
{
namespace core
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

}  // namespace core
}  // namespace romea
