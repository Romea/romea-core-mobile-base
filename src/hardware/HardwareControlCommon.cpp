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


// std
#include <limits>
#include <string>

// local
#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"


namespace romea
{
namespace core
{


//-----------------------------------------------------------------------------
std::string toCommandType(RotationalMotionControlType type)
{
  switch (type) {
    case RotationalMotionControlType::VELOCITY:
      return "angular velocity";
    case RotationalMotionControlType::TORQUE:
      return "torque";
    default:
      throw std::runtime_error("Unable to convert rotational motion control type to string");
  }
}

//-----------------------------------------------------------------------------
RotationalMotionState::RotationalMotionState()
: position(std::numeric_limits<double>::quiet_NaN()),
  velocity(std::numeric_limits<double>::quiet_NaN()),
  torque(std::numeric_limits<double>::quiet_NaN())
{
}

//-----------------------------------------------------------------------------
RotationalMotionState::RotationalMotionState(
  const double & position_,
  const double & velocity_,
  const double & torque_)
: position(position_),
  velocity(velocity_),
  torque(torque_)
{
}


//-----------------------------------------------------------------------------
std::string toCommandType(LinearMotionControlType type)
{
  switch (type) {
    case LinearMotionControlType::VELOCITY:
      return "linear velocity";
    case LinearMotionControlType::FORCE:
      return "force";
    default:
      throw std::runtime_error("Unable to convert linear motion control type to string");
  }
}

//-----------------------------------------------------------------------------
LinearMotionState::LinearMotionState()
: position(std::numeric_limits<double>::quiet_NaN()),
  velocity(std::numeric_limits<double>::quiet_NaN()),
  force(std::numeric_limits<double>::quiet_NaN())
{
}

//-----------------------------------------------------------------------------
LinearMotionState::LinearMotionState(
  const double & position_,
  const double & velocity_,
  const double & force_)
: position(position_),
  velocity(velocity_),
  force(force_)
{
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const RotationalMotionState & state)
{
  os << " position ";
  os << state.position;
  os << ", velocity ";
  os << state.velocity;
  os << ", torque ";
  os << state.torque;
  return os;
}

//-----------------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const LinearMotionState & state)
{
  os << " position ";
  os << state.position;
  os << ", velocity ";
  os << state.velocity;
  os << ", force ";
  os << state.force;
  return os;
}

}  // namespace core
}  // namespace romea
