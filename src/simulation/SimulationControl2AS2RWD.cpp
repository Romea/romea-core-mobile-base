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


#include "romea_core_mobile_base/simulation/SimulationControl2AS2RWD.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SimulationCommand2AS2RWD toSimulationCommand2AS2RWD(
  const HardwareCommand2AS2RWD & hardwareCommand,
  const double & frontLeftWheelSteeringAngle,
  const double & frontRightWheelSteeringAngle,
  const double & rearLeftWheelSteeringAngle,
  const double & rearRightWheelSteeringAngle,
  const double & frontLeftWheelSpinningSetPoint,
  const double & frontRightWheelSpinningSetPoint)
{
  return {hardwareCommand.frontAxleSteeringAngle,
    frontLeftWheelSteeringAngle,
    frontRightWheelSteeringAngle,
    hardwareCommand.rearAxleSteeringAngle,
    rearLeftWheelSteeringAngle,
    rearRightWheelSteeringAngle,
    frontLeftWheelSpinningSetPoint,
    frontRightWheelSpinningSetPoint,
    hardwareCommand.rearLeftWheelSpinningSetPoint,
    hardwareCommand.rearRightWheelSpinningSetPoint};
}

//-----------------------------------------------------------------------------
SimulationCommand2AS2RWD toSimulationCommand2AS2RWD(
  const double & /* wheelbase */,
  const double & /* frontTrack */,
  const double & /* frontWheelRadius */,
  const double & /* frontHubCarrierOffset */,
  const double & /* rearTrack */,
  const double & /* rearWheelRadius */,
  const double & /* rearHubCarrierOffset */,
  const HardwareCommand2AS2RWD & /* hardwareCommand */)
{
  //  const double & rearLeftWheelLinearSpeed =
  //      hardwareCommand.rearLeftWheelSpinningSetPoint*rearWheelRadius;

  //  const double & rearRightWheelLinearSpeed =
  //      hardwareCommand.rearRightWheelSpinningSetPoint*rearWheelRadius;

  //  double  tanFrontAxleSteeringAngle=
  //      std::tan(hardwareCommand.frontAxleSteeringAngle);

  //  double  tanRearAxleSteeringAngle=
  //      std::tan(hardwareCommand.rearAxleSteeringAngle);

  //  double frontIntantaneousCurvature = OneAxleSteeringKinematic::
  //      computeInstantaneousCurvature(tanFrontAxleSteeringAngle,wheelbase);

  //  double rearIntantaneousCurvature = OneAxleSteeringKinematic::
  //      computeInstantaneousCurvature(tanRearAxleSteeringAngle,wheelbase);

  //  double frontLeftWheelSteeringAngle = TwoWheelSteeringKinematic::
  //      computeLeftWheelSteeringAngle(tanFrontAxleSteeringAngle,
  //                                    frontIntantaneousCurvature,
  //                                    frontTrack/2.);

  //  double frontRightWheelSteeringAngle = TwoWheelSteeringKinematic::
  //      computeRightWheelSteeringAngle(tanFrontAxleSteeringAngle,
  //                                     frontIntantaneousCurvature,
  //                                     frontTrack/2.);

  //  double rearLeftWheelSteeringAngle = TwoWheelSteeringKinematic::
  //      computeLeftWheelSteeringAngle(tanRearAxleSteeringAngle,
  //                                    rearIntantaneousCurvature,
  //                                    rearTrack/2.);

  //  double rearRightWheelSteeringAngle = TwoWheelSteeringKinematic::
  //      computeRightWheelSteeringAngle(tanRearAxleSteeringAngle,
  //                                     rearIntantaneousCurvature,
  //                                     frontTrack/2.);


  //  return toSimulationCommand2AS2RWD(hardwareCommand,
  //                                    frontLeftWheelSteeringAngle,
  //                                    frontRightWheelSteeringAngle,
  //                                    rearLeftWheelSteeringAngle,
  //                                    rearRightWheelSteeringAngle);

  return {};
}

//-----------------------------------------------------------------------------
HardwareState2AS2RWD toHardwareState2AS2RWD(
  const SimulationState2AS2RWD & simulationState,
  const double frontAxleSteeringAngle,
  const double rearAxleSteeringAngle)
{
  return {frontAxleSteeringAngle,
    rearAxleSteeringAngle,
    simulationState.rearLeftWheelSpinningMotion,
    simulationState.rearRightWheelSpinningMotion};
}

//-----------------------------------------------------------------------------
HardwareState2AS2RWD toHardwareState2AS2RWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & rearTrack,
  const SimulationState2AS2RWD & simulationState)
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

  return toHardwareState2AS2RWD(
    simulationState,
    frontAxleSteeringAngle,
    rearAxleSteeringAngle);
}

}  // namespace romea
