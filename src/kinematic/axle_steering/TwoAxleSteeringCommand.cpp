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


#include <romea_core_common/math/Algorithm.hpp>
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
TwoAxleSteeringCommand::TwoAxleSteeringCommand()
: longitudinalSpeed(0.),
  frontSteeringAngle(0.),
  rearSteeringAngle(0.)
{
}

//-----------------------------------------------------------------------------
TwoAxleSteeringCommand::TwoAxleSteeringCommand(
  const double & longitudinalSpeed,
  const double & frontSteeringAngle,
  const double & rearSteeringAngle)
: longitudinalSpeed(longitudinalSpeed),
  frontSteeringAngle(frontSteeringAngle),
  rearSteeringAngle(rearSteeringAngle)
{
}

//-----------------------------------------------------------------------------
TwoAxleSteeringCommand clamp(
  const TwoAxleSteeringCommand & command,
  const TwoAxleSteeringCommandLimits & limits)
{
  TwoAxleSteeringCommand clamped_command = command;
  clamped_command.longitudinalSpeed = clamp(command.longitudinalSpeed, limits.longitudinalSpeed);
  clamped_command.frontSteeringAngle = clamp(command.frontSteeringAngle, limits.frontSteeringAngle);
  clamped_command.rearSteeringAngle = clamp(command.rearSteeringAngle, limits.rearSteeringAngle);
  return clamped_command;
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const TwoAxleSteeringCommand & command)
{
  os << " TwoAxleSteering Command " << std::endl;
  os << " command longitudinal speed  " << command.longitudinalSpeed << std::endl;
  os << " command front steering angle " << command.frontSteeringAngle << std::endl;
  os << " command rear steering angle " << command.rearSteeringAngle << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
bool isValid(const TwoAxleSteeringCommand & command)
{
  return std::isfinite(command.longitudinalSpeed) &&
         std::isfinite(command.frontSteeringAngle) &&
         std::isfinite(command.rearSteeringAngle);
}

}  // namespace romea


// old codes
////-----------------------------------------------------------------------------
// KinematicCommand toKinematicCommand(const TwoAxleSteeringCommand & command,
//                                    const double & frontWheelBase,
//                                    const double & rearWheeBase)
//{
//  double othogonalInstantaneousCurvature=TwoAxleSteeringKinematic::
//      computeOrthogonalInstantaneousCurvature(std::tan(command.frontSteeringAngle),
//                                              std::tan(command.rearSteeringAngle),
//                                              frontWheelBase,
//                                              rearWheeBase);

//  double beta= TwoAxleSteeringKinematic::computeBeta(std::tan(command.frontSteeringAngle),
//                                                     std::tan(command.rearSteeringAngle),
//                                                     frontWheelBase,
//                                                     rearWheeBase);

//  double instantaneousCurvature = othogonalInstantaneousCurvature*std::cos(beta);


//  KinematicCommand convertedCommand;
//  convertedCommand.speed=command.longitudinalSpeed/std::cos(beta);
//  convertedCommand.beta=beta;
//  convertedCommand.angularSpeed =othogonalInstantaneousCurvature*command.longitudinalSpeed;
//  convertedCommand.instantaneousCurvature =instantaneousCurvature;
//  return convertedCommand;
//}


////-----------------------------------------------------------------------------
// KinematicCommand toKinematicCommand(const TwoAxleSteeringCommand & command,
//                                    const Kinematic & kinematic)
//{
//  return toKinematicCommand(command,
//                            kinematic.getWheelBase("front_wheelbase"),
//                            kinematic.getWheelBase("rear_wheelbase"));
//}


////-----------------------------------------------------------------------------
// TwoAxleSteeringCommand toTwoAxleSteeringCommand(const KinematicCommand & command,
//                                                const double & frontWheelBase,
//                                                const double & rearWheelBase)
//{
//  double frontSteeringAngle = FourWheelSteeringKinematic::
//      computeFrontSteeringAngle(command.instantaneousCurvature,
//                                frontWheelBase,
//                                command.beta);

//  double rearSteeringAngle = FourWheelSteeringKinematic::
//      computeRearSteeringAngle(command.instantaneousCurvature,
//                               rearWheelBase,
//                               command.beta);

//  TwoAxleSteeringCommand convertedCommand;
//  convertedCommand.longitudinalSpeed=command.speed*std::cos(command.beta);
//  convertedCommand.frontSteeringAngle=frontSteeringAngle;
//  convertedCommand.rearSteeringAngle=rearSteeringAngle;
//  return convertedCommand;
//}

////-----------------------------------------------------------------------------
// TwoAxleSteeringCommand toTwoAxleSteeringCommand(const KinematicCommand & command,
//                                                const Kinematic & kinematic)
//{
//  return toTwoAxleSteeringCommand(command,
//                                  kinematic.getWheelBase("front_wheelbase"),
//                                  kinematic.getWheelBase("rear_wheelbase"));
//}
