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


#ifndef ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL1FAS4WD_HPP_
#define ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL1FAS4WD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

namespace romea
{
namespace core
{

struct HardwareState1FAS4WD
{
  SteeringAngleState frontAxleSteeringAngle;
  RotationalMotionState frontLeftWheelSpinningMotion;
  RotationalMotionState frontRightWheelSpinningMotion;
  RotationalMotionState rearLeftWheelSpinningMotion;
  RotationalMotionState rearRightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream & os, const HardwareState1FAS4WD & state);

struct HardwareCommand1FAS4WD
{
  SteeringAngleCommand frontAxleSteeringAngle;
  RotationalMotionCommand frontLeftWheelSpinningSetPoint;
  RotationalMotionCommand frontRightWheelSpinningSetPoint;
  RotationalMotionCommand rearLeftWheelSpinningSetPoint;
  RotationalMotionCommand rearRightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream & os, const HardwareState1FAS4WD & command);


}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROL1FAS4WD_HPP_
