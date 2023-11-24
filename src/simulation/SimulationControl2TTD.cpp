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
#include <cmath>

// romea
#include "romea_core_mobile_base/simulation/SimulationControl2TTD.hpp"

namespace
{

romea::core::RotationalMotionState toHardwareSprocketSpinMotion(
  const double & sprocketWheelRadius,
  const double & rollerWheelRadius,
  const double & trackThickness,
  const romea::core::RotationalMotionState & sprocketWheelSpinningMotion,
  const romea::core::RotationalMotionState & frontRollerWheelSpinningMotion,
  const romea::core::RotationalMotionState & rearRollerWheelSpinningMotion)
{
  const double sprocketWheelVirtualRadius = sprocketWheelRadius + trackThickness;
  const double rollerWheelVirtualRadius = rollerWheelRadius + trackThickness;
  const double ratio = rollerWheelVirtualRadius / sprocketWheelVirtualRadius;

  const double frontRollerWheelLinearSpeed =
    rollerWheelVirtualRadius * frontRollerWheelSpinningMotion.velocity;
  const double rearRollerWheelLinearSpeed =
    rollerWheelVirtualRadius * rearRollerWheelSpinningMotion.velocity;

  romea::core::RotationalMotionState output;
  output.position = sprocketWheelSpinningMotion.position;
  if (std::signbit(frontRollerWheelLinearSpeed) != std::signbit(rearRollerWheelLinearSpeed)) {
    output.velocity = 0;
    output.torque = 0;
  } else if (std::abs(frontRollerWheelLinearSpeed) < std::abs(rearRollerWheelLinearSpeed)) {
    output.velocity = frontRollerWheelSpinningMotion.velocity * ratio;
    output.torque = frontRollerWheelSpinningMotion.torque * ratio;
  } else {
    output.velocity = rearRollerWheelSpinningMotion.velocity * ratio;
    output.torque = rearRollerWheelSpinningMotion.torque * ratio;
  }
  return output;
}

}  // namespace

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const SimulationCommand2TTD & command)
{
  os << " Simulation2THD command : " << std::endl;
  os << " left sprocket wheel spinning setpoint : " <<
    command.leftSprocketWheelSpinningSetPoint << std::endl;
  os << " right sprocket wheel spinning setpoint : " <<
    command.rightSprocketWheelSpinningSetPoint << std::endl;
  os << " left idler wheel spinning setpoint : " <<
    command.leftIdlerWheelSpinningSetPoint << std::endl;
  os << " right idler wheel spinning setpoint : " <<
    command.rightIdlerWheelSpinningSetPoint << std::endl;
  os << " front left roller wheel spinning setpoint : " <<
    command.frontLeftRollerWheelSpinningSetPoint << std::endl;
  os << " front right roller wheel spinning setpoint : " <<
    command.frontRightRollerWheelSpinningSetPoint << std::endl;
  os << " rear left roller wheel spinning setpoint : " <<
    command.rearLeftRollerWheelSpinningSetPoint << std::endl;
  os << " rearright roller wheel spinning setpoint : " <<
    command.rearRightRollerWheelSpinningSetPoint << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const SimulationState2TTD & state)
{
  os << " Simulation2THD state : " << std::endl;
  os << " left sprocket wheel spinning motion: " <<
    state.leftSprocketWheelSpinningMotion << std::endl;
  os << " right sprocket wheel spinning motion : " <<
    state.rightSprocketWheelSpinningMotion << std::endl;
  os << " left idler wheel spinning motion: " <<
    state.leftIdlerWheelSpinningMotion << std::endl;
  os << " right idler wheel spinning motion : " <<
    state.rightIdlerWheelSpinningMotion << std::endl;
  os << " front left  roller wheel spinning motion: " <<
    state.frontLeftRollerWheelSpinningMotion << std::endl;
  os << " front right roller wheel spinning motion : " <<
    state.frontRightRollerWheelSpinningMotion << std::endl;
  os << " rear left idler roller wheel spinning motion: " <<
    state.rearLeftRollerWheelSpinningMotion << std::endl;
  os << " rear right idler roller wheel spinning motion : " <<
    state.rearRightRollerWheelSpinningMotion;
  return os;
}

//-----------------------------------------------------------------------------
SimulationCommand2TTD toSimulationCommand2TTD(
  const double & sprocketWheelRadius,
  const double & rollerWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const HardwareCommand2TD & hardwareCommand)
{
  const double idler_ratio =
    (sprocketWheelRadius + trackThickness) / (idlerWheelRadius + trackThickness);
  const double roller_ratio =
    (sprocketWheelRadius + trackThickness) / (rollerWheelRadius + trackThickness);

  return {hardwareCommand.leftSprocketWheelSpinningSetPoint,
    hardwareCommand.rightSprocketWheelSpinningSetPoint,
    hardwareCommand.leftSprocketWheelSpinningSetPoint * idler_ratio,
    hardwareCommand.rightSprocketWheelSpinningSetPoint * idler_ratio,
    hardwareCommand.leftSprocketWheelSpinningSetPoint * roller_ratio,
    hardwareCommand.rightSprocketWheelSpinningSetPoint * roller_ratio,
    hardwareCommand.leftSprocketWheelSpinningSetPoint * roller_ratio,
    hardwareCommand.rightSprocketWheelSpinningSetPoint * roller_ratio};
}

//-----------------------------------------------------------------------------
HardwareState2TD toHardwareState2TTD(
  const double & sprocketWheelRadius,
  const double & rollerWheelRadius,
  const double & trackThickness,
  const SimulationState2TTD & simulationState)
{
  return {toHardwareSprocketSpinMotion(
      sprocketWheelRadius,
      rollerWheelRadius,
      trackThickness,
      simulationState.leftSprocketWheelSpinningMotion,
      simulationState.frontLeftRollerWheelSpinningMotion,
      simulationState.rearLeftRollerWheelSpinningMotion),
    toHardwareSprocketSpinMotion(
      sprocketWheelRadius,
      rollerWheelRadius,
      trackThickness,
      simulationState.rightSprocketWheelSpinningMotion,
      simulationState.frontRightRollerWheelSpinningMotion,
      simulationState.rearRightRollerWheelSpinningMotion)};
}

}  // namespace core
}  // namespace romea
