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


#include "romea_core_mobile_base/simulation/SimulationControl2AS2FWD.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
SimulationCommand2AS2FWD toSimulationCommand2AS2FWD(
  const HardwareCommand2AS2FWD & hardwareCommand,
  const double & frontLeftWheelSteeringAngle,
  const double & frontRightWheelSteeringAngle,
  const double & rearLeftWheelSteeringAngle,
  const double & rearRightWheelSteeringAngle,
  const double & rearLeftWheelSpinningSetPoint,
  const double & rearRightWheelSpinningSetPoint)
{
  return {hardwareCommand.frontAxleSteeringAngle,
    frontLeftWheelSteeringAngle,
    frontRightWheelSteeringAngle,
    hardwareCommand.rearAxleSteeringAngle,
    rearLeftWheelSteeringAngle,
    rearRightWheelSteeringAngle,
    hardwareCommand.frontLeftWheelSpinningSetPoint,
    hardwareCommand.frontRightWheelSpinningSetPoint,
    rearLeftWheelSpinningSetPoint,
    rearRightWheelSpinningSetPoint};
}

//-----------------------------------------------------------------------------
SimulationCommand2AS2FWD toSimulationCommand2AS2FWD(
  const double & /* wheelbase */,
  const double & /* frontTrack */,
  const double & /* frontWheelRadius */,
  const double & /* frontHubCarrierOffset */,
  const double & /* rearTrack */,
  const double & /* rearWheelRadius */,
  const double & /* rearHubCarrierOffset */,
  const HardwareCommand2AS2FWD & /* hardwareCommand */)
{
//  double  tanFrontAxleSteeringAngle=
//      std::tan(hardwareCommand.frontAxleSteeringAngle);

//  double  tanRearAxleSteeringAngle=
//      std::tan(hardwareCommand.rearAxleSteeringAngle);

//  double frontIntantaneousCurvature = OneAxleSteeringKinematic::
//      computeInstantaneousCurvature(tanFrontAxleSteeringAngle,wheelbase);

//  double rearIntantaneousCurvature = OneAxleSteeringKinematic::
//      computeInstantaneousCurvature(tanRearAxleSteeringAngle,wheelbase);

//  double frontLeftWheelAngle = TwoWheelSteeringKinematic::
//      computeLeftWheelSteeringAngle(tanFrontAxleSteeringAngle,
//                                    frontIntantaneousCurvature,
//                                    frontTrack/2.);

//  double frontRightWheelAngle = TwoWheelSteeringKinematic::
//      computeRightWheelSteeringAngle(tanFrontAxleSteeringAngle,
//                                     frontIntantaneousCurvature,
//                                     frontTrack/2.);

//  double rearLeftWheelAngle = TwoWheelSteeringKinematic::
//      computeLeftWheelSteeringAngle(tanRearAxleSteeringAngle,
//                                    rearIntantaneousCurvature,
//                                    rearTrack/2.);

//  double rearRightWheelAngle = TwoWheelSteeringKinematic::
//      computeRightWheelSteeringAngle(tanRearAxleSteeringAngle,
//                                     rearIntantaneousCurvature,
//                                     frontTrack/2.);

//  const double & frontLeftWheelLinearSpeed =
//      hardwareCommand.frontLeftWheelSpinningSetPoint*frontWheelRadius;

//  const double & frontRightWheelLinearSpeed =
//      hardwareCommand.frontRightWheelSpinningSetPoint*frontWheelRadius;


//  double linearSpeed = 0.5 * (frontLeftWheelLinearSpeed / TwoWheelSteeringKinematic::
//                              computeWheelLinearSpeedRatio(-tanFrontAxleSteeringAngle,
//                                                           -instantaneousCurvature,
//                                                           frontHubCarrierOffset,
//                                                           frontTrack/2.)+
//                              frontRightWheelLinearSpeed / TwoWheelSteeringKinematic::
//                              computeWheelLinearSpeedRatio(tanFrontAxleSteeringAngle,
//                                                           instantaneousCurvature,
//                                                           frontHubCarrierOffset,
//                                                           frontTrack/2.));

//  double rearLeftWheelLinearSpeed = TwoWheelSteeringKinematic::
//      computeLeftWheelLinearSpeed(linearSpeedCommand,
//                                  tanRearAxleSteeringAngle,
//                                  intantaneousCurvature,
//                                  frontHubCarrierOffset,
//                                  frontTrack/2.);

//  double rearRightWheelLinearSpeed = TwoWheelSteeringKinematic::
//      computeRightWheelLinearSpeed(linearSpeedCommand,
//                                   tanRearAxleSteeringAngle,
//                                   intantaneousCurvature,
//                                   frontHubCarrierOffset,
//                                   frontTrack/2.);

  //  return toSimulationCommand2AS4WD(hardwareCommand,
  //                                   frontLeftWheelAngle,
  //                                   frontRightWheelAngle,
  //                                   rearLeftWheelAngle,
  //                                   rearRightWheelAngle,
  //                                   frontLeftWheelLinearSpeed,
  //                                   frontRightWheelLinearSpeed);

  return {};
}

//-----------------------------------------------------------------------------
HardwareState2AS2FWD toHardwareState2AS2FWD(
  const SimulationState2AS2FWD & simulationState,
  const double frontAxleSteeringAngle,
  const double rearAxleSteeringAngle)
{
  return {frontAxleSteeringAngle,
    rearAxleSteeringAngle,
    simulationState.frontLeftWheelSpinningMotion,
    simulationState.frontRightWheelSpinningMotion};
}

//-----------------------------------------------------------------------------
HardwareState2AS2FWD toHardwareState2AS2FWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & rearTrack,
  const SimulationState2AS2FWD & simulationState)
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

  return toHardwareState2AS2FWD(
    simulationState,
    frontAxleSteeringAngle,
    rearAxleSteeringAngle);
}

}  // namespace core
}  // namespace romea
