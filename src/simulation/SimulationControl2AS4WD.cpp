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


#include "romea_core_mobile_base/simulation/SimulationControl2AS4WD.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SimulationCommand2AS4WD toSimulationCommand2AS4WD(
  const HardwareCommand2AS4WD & hardwareCommand,
  const double & frontLeftWheelSteeringAngle,
  const double & frontRightWheelSteeringAngle,
  const double & rearLeftWheelSteeringAngle,
  const double & rearRightWheelSteeringAngle)
{
  return {hardwareCommand.frontAxleSteeringAngle,
    frontLeftWheelSteeringAngle,
    frontRightWheelSteeringAngle,
    hardwareCommand.rearAxleSteeringAngle,
    rearLeftWheelSteeringAngle,
    rearRightWheelSteeringAngle,
    hardwareCommand.frontLeftWheelSpinningSetPoint,
    hardwareCommand.frontRightWheelSpinningSetPoint,
    hardwareCommand.rearLeftWheelSpinningSetPoint,
    hardwareCommand.rearRightWheelSpinningSetPoint};
}

//-----------------------------------------------------------------------------
SimulationCommand2AS4WD toSimulationCommand2AS4WD(
  const double & wheelbase,
  const double & frontTrack,
  const double & rearTrack,
  const HardwareCommand2AS4WD & hardwareCommand)
{
  double tanFrontAxleSteeringAngle = std::tan(hardwareCommand.frontAxleSteeringAngle);
  double tanRearAxleSteeringAngle = std::tan(hardwareCommand.rearAxleSteeringAngle);

  double frontIntantaneousCurvature = OneAxleSteeringKinematic::
    computeInstantaneousCurvature(tanFrontAxleSteeringAngle, wheelbase);

  double rearIntantaneousCurvature = OneAxleSteeringKinematic::
    computeInstantaneousCurvature(tanRearAxleSteeringAngle, wheelbase);

  double frontLeftWheelAngle = TwoWheelSteeringKinematic::
    computeLeftWheelSteeringAngle(
    tanFrontAxleSteeringAngle,
    frontIntantaneousCurvature,
    frontTrack / 2.);

  double frontRightWheelAngle = TwoWheelSteeringKinematic::
    computeRightWheelSteeringAngle(
    tanFrontAxleSteeringAngle,
    frontIntantaneousCurvature,
    frontTrack / 2.);

  double rearLeftWheelAngle = TwoWheelSteeringKinematic::
    computeLeftWheelSteeringAngle(
    tanRearAxleSteeringAngle,
    rearIntantaneousCurvature,
    rearTrack / 2.);

  double rearRightWheelAngle = TwoWheelSteeringKinematic::
    computeRightWheelSteeringAngle(
    tanRearAxleSteeringAngle,
    rearIntantaneousCurvature,
    frontTrack / 2.);

  return toSimulationCommand2AS4WD(
    hardwareCommand,
    frontLeftWheelAngle,
    frontRightWheelAngle,
    rearLeftWheelAngle,
    rearRightWheelAngle);
}

//-----------------------------------------------------------------------------
HardwareState2AS4WD toHardwareState2AS4WD(
  const SimulationState2AS4WD & simulationState,
  const double frontAxleSteeringAngle,
  const double rearAxleSteeringAngle)
{
  return {frontAxleSteeringAngle,
    rearAxleSteeringAngle,
    simulationState.frontLeftWheelSpinningMotion,
    simulationState.frontRightWheelSpinningMotion,
    simulationState.rearLeftWheelSpinningMotion,
    simulationState.rearRightWheelSpinningMotion};
}

//-----------------------------------------------------------------------------
HardwareState2AS4WD toHardwareState2AS4WD(
  const double & wheelbase,
  const double & frontTrack,
  const double & rearTrack,
  const SimulationState2AS4WD & simulationState)
{
  double frontAxleSteeringAngle = TwoWheelSteeringKinematic::
    computeSteeringAngle(
    simulationState.frontLeftWheelSteeringAngle,
    simulationState.frontRightWheelSteeringAngle,
    wheelbase, frontTrack);

  double rearAxleSteeringAngle = TwoWheelSteeringKinematic::
    computeSteeringAngle(
    simulationState.rearLeftWheelSteeringAngle,
    simulationState.rearRightWheelSteeringAngle,
    wheelbase, rearTrack);

  return toHardwareState2AS4WD(
    simulationState,
    frontAxleSteeringAngle,
    rearAxleSteeringAngle);
}

}  // namespace romea
