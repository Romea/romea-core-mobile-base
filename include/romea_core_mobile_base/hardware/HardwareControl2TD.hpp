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


#ifndef ROMEA_CORE_MOBILE__BASE__HARDWARE_HARDWARECONTROL2TD_HPP_
#define ROMEA_CORE_MOBILE__BASE__HARDWARE_HARDWARECONTROL2TD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

namespace romea
{
namespace core
{

struct HardwareCommand2TD
{
  RotationalMotionCommand leftSprocketWheelSpinningSetPoint;
  RotationalMotionCommand rightSprocketWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream & os, const HardwareCommand2TD & command);

struct HardwareState2TD
{
  RotationalMotionState leftSprocketWheelSpinningMotion;
  RotationalMotionState rightSprocketWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream & os, const HardwareState2TD & state);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROL2TD_HPP_
