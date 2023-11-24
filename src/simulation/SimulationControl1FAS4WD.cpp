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


#include "romea_core_mobile_base/simulation/SimulationControl1FAS4WD.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
SimulationCommand1FAS4WD toSimulationCommand1FAS4WD(
  const HardwareCommand1FAS4WD & hardwareCommand,
  const double & frontLeftWheelSteeringAngle,
  const double & frontRightWheelSteeringAngle)
{
  return {hardwareCommand.frontAxleSteeringAngle,
    frontLeftWheelSteeringAngle,
    frontRightWheelSteeringAngle,
    hardwareCommand.frontLeftWheelSpinningSetPoint,
    hardwareCommand.frontRightWheelSpinningSetPoint,
    hardwareCommand.rearLeftWheelSpinningSetPoint,
    hardwareCommand.rearRightWheelSpinningSetPoint};
}

//-----------------------------------------------------------------------------
SimulationCommand1FAS4WD toSimulationCommand1FAS2RWD(
  const double & wheelbase,
  const double & frontTrack,
  const HardwareCommand1FAS4WD & hardwareCommand)
{
  double tanAxleSteeringAngle = std::tan(hardwareCommand.frontAxleSteeringAngle);

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

  return toSimulationCommand1FAS4WD(
    hardwareCommand,
    frontLeftWheelSteeringAngle,
    frontRightWheelSteeringAngle);
}

//-----------------------------------------------------------------------------
HardwareState1FAS4WD toHardwareState1FAS4WD(
  const SimulationState1FAS4WD & simulationState,
  const double frontAxleSteeringAngle)
{
  return {frontAxleSteeringAngle,
    simulationState.frontLeftWheelSpinningMotion,
    simulationState.frontRightWheelSpinningMotion,
    simulationState.rearLeftWheelSpinningMotion,
    simulationState.rearRightWheelSpinningMotion};
}

//-----------------------------------------------------------------------------
HardwareState1FAS4WD toHardwareState1FAS4WD(
  const double & wheelbase,
  const double & frontTrack,
  const SimulationState1FAS4WD & simulationState)
{
  double frontAxleSteeringAngle = TwoWheelSteeringKinematic::
    computeSteeringAngle(
    simulationState.frontLeftWheelSteeringAngle,
    simulationState.frontRightWheelSteeringAngle,
    wheelbase, frontTrack);

  return toHardwareState1FAS4WD(simulationState, frontAxleSteeringAngle);
}

}  // namespace core
}  // namespace romea
