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


#include "romea_core_mobile_base/simulation/SimulationControl1FAS2RWD.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
SimulationCommand1FAS2RWD toSimulationCommand1FAS2RWD(
  const HardwareCommand1FAS2RWD & hardwareCommand,
  const double & frontLeftWheelSteeringAngle,
  const double & frontRightWheelSteeringAngle,
  const double & frontLeftWheelSpinningSetPoint,
  const double & frontRightWheelSpinningSetPoint)
{
  return {hardwareCommand.frontAxleSteeringAngle,
    frontLeftWheelSteeringAngle,
    frontRightWheelSteeringAngle,
    frontLeftWheelSpinningSetPoint,
    frontRightWheelSpinningSetPoint,
    hardwareCommand.rearLeftWheelSpinningSetPoint,
    hardwareCommand.rearRightWheelSpinningSetPoint};
}

//-----------------------------------------------------------------------------
SimulationCommand1FAS2RWD toSimulationCommand1FAS2RWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & frontHubCarrierOffset,
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const HardwareCommand1FAS2RWD & hardwareCommand)
{
  const double & rearLeftWheelLinearSpeed =
    hardwareCommand.rearLeftWheelSpinningSetPoint * rearWheelRadius;

  const double & rearRightWheelLinearSpeed =
    hardwareCommand.rearRightWheelSpinningSetPoint * rearWheelRadius;

  double tanAxleSteeringAngle =
    std::tan(hardwareCommand.frontAxleSteeringAngle);

  double intantaneousCurvature = OneAxleSteeringKinematic::
    computeInstantaneousCurvature(tanAxleSteeringAngle, wheelbase);


  double frontLeftWheelSteeringAngle = TwoWheelSteeringKinematic::
    computeLeftWheelSteeringAngle(
    tanAxleSteeringAngle,
    intantaneousCurvature,
    frontTrack / 2.);

  double frontRightWheelSteeringAngle = TwoWheelSteeringKinematic::
    computeRightWheelSteeringAngle(
    tanAxleSteeringAngle,
    intantaneousCurvature,
    frontTrack / 2.);

  double linearSpeedCommand = (rearLeftWheelLinearSpeed + rearRightWheelLinearSpeed) / 2.;

  double frontLeftWheelLinearSpeed = TwoWheelSteeringKinematic::
    computeLeftWheelLinearSpeed(
    linearSpeedCommand,
    tanAxleSteeringAngle,
    intantaneousCurvature,
    frontHubCarrierOffset,
    frontTrack / 2.);

  double frontRightWheelLinearSpeed = TwoWheelSteeringKinematic::
    computeRightWheelLinearSpeed(
    linearSpeedCommand,
    tanAxleSteeringAngle,
    intantaneousCurvature,
    frontHubCarrierOffset,
    frontTrack / 2.);


  return toSimulationCommand1FAS2RWD(
    hardwareCommand,
    frontLeftWheelSteeringAngle,
    frontRightWheelSteeringAngle,
    frontLeftWheelLinearSpeed / frontWheelRadius,
    frontRightWheelLinearSpeed / frontWheelRadius);
}

//-----------------------------------------------------------------------------
SimulationState1FAS2RWD toSimulationState1FAS2RWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & frontHubCarrierOffset,
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const HardwareState1FAS2RWD & hardwareState)
{
  core::HardwareCommand1FAS2RWD fakeHardwareCommand = {
    hardwareState.frontAxleSteeringAngle,
    hardwareState.rearLeftWheelSpinningMotion.velocity,
    hardwareState.rearRightWheelSpinningMotion.velocity
  };

  SimulationCommand1FAS2RWD fakeSimulationCommand = toSimulationCommand1FAS2RWD(
    wheelbase,
    frontTrack,
    frontHubCarrierOffset,
    frontWheelRadius,
    rearWheelRadius,
    fakeHardwareCommand);

  SimulationState1FAS2RWD simulationState;
  simulationState.frontAxleSteeringAngle =
    hardwareState.frontAxleSteeringAngle;
  simulationState.frontLeftWheelSteeringAngle =
    fakeSimulationCommand.frontLeftWheelSteeringAngle;
  simulationState.frontRightWheelSteeringAngle =
    fakeSimulationCommand.frontRightWheelSteeringAngle;
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
HardwareState1FAS2RWD toHardwareState1FAS2RWD(
  const SimulationState1FAS2RWD & simulationState,
  const double frontAxleSteeringAngle)
{
  return {frontAxleSteeringAngle,
    simulationState.rearLeftWheelSpinningMotion,
    simulationState.rearRightWheelSpinningMotion};
}

//-----------------------------------------------------------------------------
HardwareState1FAS2RWD toHardwareState1FAS2RWD(
  const double & wheelbase,
  const double & frontTrack,
  const SimulationState1FAS2RWD & simulationState)
{
  double frontAxleSteeringAngle = TwoWheelSteeringKinematic::
    computeSteeringAngle(
    simulationState.frontLeftWheelSteeringAngle,
    simulationState.frontRightWheelSteeringAngle,
    wheelbase, frontTrack);

  return toHardwareState1FAS2RWD(simulationState, frontAxleSteeringAngle);
}

}  // namespace core
}  // namespace romea
