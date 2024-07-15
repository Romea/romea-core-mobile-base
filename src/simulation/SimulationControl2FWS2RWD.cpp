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


#include "romea_core_mobile_base/simulation/SimulationControl2FWS2RWD.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
SimulationCommand2FWS2RWD toSimulationCommand2FWS2RWD(
  const HardwareCommand2FWS2RWD & hardwareCommand,
  const double & frontLeftWheelSpinningSetPoint,
  const double & frontRightWheelSpinningSetPoint)
{
  return {hardwareCommand.frontLeftWheelSteeringAngle,
    hardwareCommand.frontRightWheelSteeringAngle,
    frontLeftWheelSpinningSetPoint,
    frontRightWheelSpinningSetPoint,
    hardwareCommand.rearLeftWheelSpinningSetPoint,
    hardwareCommand.rearRightWheelSpinningSetPoint};
}

//-----------------------------------------------------------------------------
SimulationCommand2FWS2RWD toSimulationCommand2FWS2RWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & frontHubCarrierOffset,
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const HardwareCommand2FWS2RWD & hardwareCommand)
{
  const double & rearLeftWheelLinearSpeed =
    hardwareCommand.rearLeftWheelSpinningSetPoint * rearWheelRadius;

  const double & rearRightWheelLinearSpeed =
    hardwareCommand.rearRightWheelSpinningSetPoint * rearWheelRadius;

  const double & frontLeftWheelSteeringAngle =
    hardwareCommand.frontLeftWheelSteeringAngle;

  const double & frontRightWheelSteeringAngle =
    hardwareCommand.frontRightWheelSteeringAngle;

  double linearSpeed = (rearLeftWheelLinearSpeed + rearRightWheelLinearSpeed) / 2.0;

  double instantaneousCurvature = TwoWheelSteeringKinematic::
    computeInstantaneousCurvature(
    frontLeftWheelSteeringAngle,
    frontRightWheelSteeringAngle,
    wheelbase, frontTrack);

  double frontFeftWheelLinearSpeed = TwoWheelSteeringKinematic::
    computeLeftWheelLinearSpeed(
    linearSpeed,
    instantaneousCurvature * wheelbase,
    instantaneousCurvature,
    frontHubCarrierOffset,
    frontTrack / 2.0);

  double frontRightWheelLinearSpeed = TwoWheelSteeringKinematic::
    computeRightWheelLinearSpeed(
    linearSpeed,
    instantaneousCurvature * wheelbase,
    instantaneousCurvature,
    frontHubCarrierOffset,
    frontTrack / 2.0);

  return toSimulationCommand2FWS2RWD(
    hardwareCommand,
    frontFeftWheelLinearSpeed / frontWheelRadius,
    frontRightWheelLinearSpeed / frontWheelRadius);
}

//-----------------------------------------------------------------------------
SimulationState2FWS2RWD  toSimulationState2FWS2RWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & frontHubCarrierOffset,
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const HardwareState2FWS2RWD & hardwareState)
{
  HardwareCommand2FWS2RWD fakeHardwareCommand = {
    hardwareState.frontLeftWheelSteeringAngle,
    hardwareState.frontRightWheelSteeringAngle,
    hardwareState.rearLeftWheelSpinningMotion.velocity,
    hardwareState.rearRightWheelSpinningMotion.velocity,
  };


  SimulationCommand2FWS2RWD fakeSimulationCommand = toSimulationCommand2FWS2RWD(
    wheelbase,
    frontTrack,
    frontWheelRadius,
    rearWheelRadius,
    frontHubCarrierOffset,
    fakeHardwareCommand);

  SimulationState2FWS2RWD simulationState;
  simulationState.frontLeftWheelSteeringAngle =
    hardwareState.frontLeftWheelSteeringAngle;
  simulationState.frontRightWheelSteeringAngle =
    hardwareState.frontRightWheelSteeringAngle;
  simulationState.frontLeftWheelSpinningMotion.velocity =
    fakeSimulationCommand.frontLeftWheelSpinningSetPoint;
  simulationState.frontRightWheelSpinningMotion.velocity =
    fakeSimulationCommand.frontRightWheelSpinningSetPoint;
  simulationState.rearLeftWheelSpinningMotion =
    hardwareState.rearLeftWheelSpinningMotion;
  simulationState.rearRightWheelSpinningMotion =
    hardwareState.rearRightWheelSpinningMotion;

  return simulationState;
}

//-----------------------------------------------------------------------------
HardwareState2FWS2RWD toHardwareState2FWS2RWD(const SimulationState2FWS2RWD & simulationState)
{
  return {simulationState.frontLeftWheelSteeringAngle,
    simulationState.frontRightWheelSteeringAngle,
    simulationState.rearLeftWheelSpinningMotion,
    simulationState.rearRightWheelSpinningMotion};
}

}  // namespace core
}  // namespace romea
