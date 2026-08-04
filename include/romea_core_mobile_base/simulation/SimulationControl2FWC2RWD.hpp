// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

#ifndef ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWC2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWC2RWD_HPP_

#include <ostream>

#include "romea_core_mobile_base/hardware/HardwareControl2FWC2RWD.hpp"

namespace romea
{
namespace core
{

struct SimulationCommand2FWC2RWD
{
  RotationalMotionCommand rearLeftWheelSpinningSetPoint;
  RotationalMotionCommand rearRightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream & os, const SimulationCommand2FWC2RWD & command);

struct SimulationState2FWC2RWD
{
  SteeringAngleState frontLeftWheelSwivelingAngle;
  SteeringAngleState frontRightWheelSwivelingAngle;

  RotationalMotionState frontLeftWheelSpinningMotion;
  RotationalMotionState frontRightWheelSpinningMotion;
  RotationalMotionState rearLeftWheelSpinningMotion;
  RotationalMotionState rearRightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream & os, const SimulationState2FWC2RWD & state);

SimulationCommand2FWC2RWD toSimulationCommand2FWC2RWD(
  const HardwareCommand2FWC2RWD & hardwareCommand);

SimulationState2FWC2RWD toSimulationState2FWC2RWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & frontWheelXOffset,
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const HardwareState2FWC2RWD & hardwareState);

HardwareState2FWC2RWD toHardwareState2FWC2RWD(const SimulationState2FWC2RWD & simulationState);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWC2RWD_HPP_
