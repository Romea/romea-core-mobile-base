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


#ifndef ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROLCOMMON_HPP_
#define ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROLCOMMON_HPP_

#include <iostream>
#include <string>

namespace romea
{
namespace core
{

using SteeringAngleCommand = double;
using SteeringAngleState = double;


struct RotationalMotionState
{
  RotationalMotionState();
  double position;
  double velocity;
  double torque;
};

using RotationalMotionCommand = double;

enum class RotationalMotionControlType
{
  VELOCITY,
  TORQUE
};

std::string toCommandType(RotationalMotionControlType type);

struct LinearMotionState
{
  LinearMotionState();
  double position;
  double velocity;
  double force;
};

using LinearMotionCommand = double;

enum class LinearMotionControlType
{
  VELOCITY,
  FORCE
};

std::ostream & operator<<(std::ostream & s, const RotationalMotionState & state);
std::ostream & operator<<(std::ostream & s, const LinearMotionState & state);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROLCOMMON_HPP_
