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


#include "romea_core_mobile_base/hardware/HardwareControl2WD.hpp"

namespace romea
{
namespace core
{

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const HardwareCommand2WD & command)
{
  os << " Hardware2WD command : " << std::endl;
  os << " left wheel spinning setpoint : ";
  os << command.leftWheelSpinningSetPoint << std::endl;
  os << " right wheel spinning setpoint : ";
  os << command.rightWheelSpinningSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const HardwareState2WD & state)
{
  os << " Hardware2WD state : " << std::endl;
  os << " left wheel rotational motion : ";
  os << state.leftWheelSpinningMotion << std::endl;
  os << " right wheel rotational motion : ";
  os << state.rightWheelSpinningMotion << std::endl;
  return os;
}

}  // namespace core
}  // namespace romea
