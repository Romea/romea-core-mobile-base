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


#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp"
#include <romea_core_common/math/Algorithm.hpp>

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
OmniSteeringCommand::OmniSteeringCommand()
: longitudinalSpeed(0.),
  lateralSpeed(0.),
  angularSpeed(0.)
{
}

//-----------------------------------------------------------------------------
OmniSteeringCommand::OmniSteeringCommand(
  const double & longitudinalSpeed,
  const double & lateralSpeed,
  const double & angularSpeed)
: longitudinalSpeed(longitudinalSpeed),
  lateralSpeed(lateralSpeed),
  angularSpeed(angularSpeed)
{
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const OmniSteeringCommand & command)
{
  os << " OmniSteering command   " << std::endl;
  os << " command longitudinal speed  " << command.longitudinalSpeed << std::endl;
  os << " command lateral speed  " << command.lateralSpeed << std::endl;
  os << " command angular speed " << command.angularSpeed << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
OmniSteeringCommand clamp(
  const OmniSteeringCommand & command,
  const OmniSteeringCommandLimits & limits)
{
  OmniSteeringCommand clampedCommand;
  clampedCommand.longitudinalSpeed = clamp(command.longitudinalSpeed, limits.longitudinalSpeed);
  clampedCommand.lateralSpeed = clamp(command.lateralSpeed, limits.lateralSpeed);
  clampedCommand.angularSpeed = clamp(command.angularSpeed, limits.angularSpeed);
  return clampedCommand;
}

//-----------------------------------------------------------------------------
bool isValid(const OmniSteeringCommand & command)
{
  return std::isfinite(command.longitudinalSpeed) &&
         std::isfinite(command.lateralSpeed) &&
         std::isfinite(command.angularSpeed);
}

}  // namespace core
}  // namespace romea
