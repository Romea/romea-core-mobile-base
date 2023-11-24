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
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
OneAxleSteeringCommand::OneAxleSteeringCommand()
: longitudinalSpeed(0.),
  steeringAngle(0.)
{
}

//-----------------------------------------------------------------------------
OneAxleSteeringCommand::OneAxleSteeringCommand(
  const double & longitudinalSpeed,
  const double & steeringAngle)
: longitudinalSpeed(longitudinalSpeed),
  steeringAngle(steeringAngle)
{
}

//-----------------------------------------------------------------------------
OneAxleSteeringCommand clamp(
  const OneAxleSteeringCommand & command,
  const OneAxleSteeringCommandLimits & limits)
{
  OneAxleSteeringCommand clamped_command = command;
  clamped_command.longitudinalSpeed = clamp(command.longitudinalSpeed, limits.longitudinalSpeed);
  clamped_command.steeringAngle = clamp(command.steeringAngle, limits.steeringAngle);
  return clamped_command;
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const OneAxleSteeringCommand & command)
{
  os << " OneAxleSteeringCommand command" << std::endl;
  os << " command longitudinal speed  " << command.longitudinalSpeed << std::endl;
  os << " command front steering angle " << command.steeringAngle << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
bool isValid(const OneAxleSteeringCommand & command)
{
  return std::isfinite(command.longitudinalSpeed) &&
         std::isfinite(command.steeringAngle);
}


}  // namespace core
}  // namespace romea


// old codes
////-----------------------------------------------------------------------------
// KinematicCommand toKinematicCommand(const OneAxleSteeringCommand & command,
//                                    const Kinematic & kinematic)
//{
//  return toKinematicCommand(command,kinematic.getWheelBase("wheelbase"));
//}

////-----------------------------------------------------------------------------
// OneAxleSteeringCommand toOneAxleSteeringCommand(const KinematicCommand & command,
//                                                const double &wheelBase)
//{
//    assert(command.beta<std::numeric_limits<double>::epsilon());
//    OneAxleSteeringCommand convertedCommand;
//    convertedCommand.longitudinalSpeed =command.speed;
//    convertedCommand.steeringAngle=std::atan(command.instantaneousCurvature*wheelBase);
//    return convertedCommand;
//}

////-----------------------------------------------------------------------------
// OneAxleSteeringCommand toOneAxleSteeringCommand(const KinematicCommand & command,
//                                                const Kinematic & kinematic)
//{
//  return toOneAxleSteeringCommand(command,kinematic.getWheelBase("wheelbase"));
//}
