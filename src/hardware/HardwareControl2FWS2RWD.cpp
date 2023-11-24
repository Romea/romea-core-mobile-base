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


#include "romea_core_mobile_base/hardware/HardwareControl2FWS2RWD.hpp"

namespace romea
{
namespace core
{

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const HardwareCommand2FWS2RWD & command)
{
  os << " Hardware2FWS2RWD command : " << std::endl;
  os << " front left wheel steering angle : ";
  os << command.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : ";
  os << command.frontRightWheelSteeringAngle << std::endl;
  os << " rear left wheel spinning setpoint : ";
  os << command.rearLeftWheelSpinningSetPoint << std::endl;
  os << " rear right wheel spinning setpoint : ";
  os << command.rearRightWheelSpinningSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const HardwareState2FWS2RWD & state)
{
  os << " Hardware2FWS2RWD state : " << std::endl;
  os << " front left wheel steering angle : ";
  os << state.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : ";
  os << state.frontRightWheelSteeringAngle << std::endl;
  os << " rear left wheel rotational  : ";
  os << state.rearLeftWheelSpinningMotion << std::endl;
  os << " rear right wheel rotational motion : ";
  os << state.rearRightWheelSpinningMotion << std::endl;
  return os;
}

}  // namespace core
}  // namespace romea
