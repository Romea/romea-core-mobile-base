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
#include "romea_core_mobile_base/simulation/SimulationControl2THD.hpp"


namespace
{

romea::core::RotationalMotionState toHardwareSprocketSpinMotion(
  const double & sprocketWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const romea::core::RotationalMotionState & sprocketWheelSpinningMotion,
  const romea::core::RotationalMotionState & frontIdlerWheelSpinningMotion,
  const romea::core::RotationalMotionState & rearIdlerWheelSpinningMotion)
{
  const double sprocketWheelVirtualRadius = sprocketWheelRadius + trackThickness;
  const double idlerWheelVirtualRadius = idlerWheelRadius + trackThickness;
  const double ratio = idlerWheelVirtualRadius / sprocketWheelVirtualRadius;

  const double frontIdlerWheelLinearSpeed =
    idlerWheelVirtualRadius * frontIdlerWheelSpinningMotion.velocity;
  const double rearIdlerWheelLinearSpeed =
    idlerWheelVirtualRadius * rearIdlerWheelSpinningMotion.velocity;

  romea::core::RotationalMotionState output;
  output.position = sprocketWheelSpinningMotion.position;
  if (std::signbit(frontIdlerWheelLinearSpeed) != std::signbit(rearIdlerWheelLinearSpeed)) {
    output.velocity = 0;
    output.torque = 0;
  } else if (std::abs(frontIdlerWheelLinearSpeed) < std::abs(rearIdlerWheelLinearSpeed)) {
    output.velocity = frontIdlerWheelSpinningMotion.velocity * ratio;
    output.torque = frontIdlerWheelSpinningMotion.torque * ratio;
  } else {
    output.velocity = rearIdlerWheelSpinningMotion.velocity * ratio;
    output.torque = rearIdlerWheelSpinningMotion.torque * ratio;
  }
  return output;
}

}  // namespace

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const SimulationCommand2THD & command)
{
  os << " Simulation2THD command : " << std::endl;
  os << " left sprocket wheel spinning setpoint : " <<
    command.leftSprocketWheelSpinningSetPoint << std::endl;
  os << " right sprocket wheel spinning setpoint : " <<
    command.rightSprocketWheelSpinningSetPoint << std::endl;
  os << " front left idler wheel spinning setpoint : " <<
    command.frontLeftIdlerWheelSpinningSetPoint << std::endl;
  os << " front right idler wheel spinning setpoint : " <<
    command.frontRightIdlerWheelSpinningSetPoint << std::endl;
  os << " rear left idler wheel spinning setpoint : " <<
    command.rearLeftIdlerWheelSpinningSetPoint << std::endl;
  os << " rearright idler wheel spinning setpoint : " <<
    command.rearRightIdlerWheelSpinningSetPoint << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const SimulationState2THD & state)
{
  os << " Simulation2THD state : " << std::endl;
  os << " left sprocket wheel spinning motion: " <<
    state.leftSprocketWheelSpinningMotion << std::endl;
  os << " right sprocket wheel spinning motion : " <<
    state.rightSprocketWheelSpinningMotion << std::endl;
  os << " front left idler wheel spinning motion: " <<
    state.frontLeftIdlerWheelSpinningMotion << std::endl;
  os << " frontright idler wheel spinning motion : " <<
    state.frontRightIdlerWheelSpinningMotion << std::endl;
  os << " rear left idler wheel spinning motion: " <<
    state.rearLeftIdlerWheelSpinningMotion << std::endl;
  os << " rear right idler wheel spinning motion : " <<
    state.rearRightIdlerWheelSpinningMotion;
  return os;
}

//-----------------------------------------------------------------------------
SimulationCommand2THD toSimulationCommand2THD(
  const double & sprocketWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const HardwareCommand2TD & hardwareCommand)
{
  const double ratio = (sprocketWheelRadius + trackThickness) / (idlerWheelRadius + trackThickness);

  return{hardwareCommand.leftSprocketWheelSpinningSetPoint,
    hardwareCommand.rightSprocketWheelSpinningSetPoint,
    hardwareCommand.leftSprocketWheelSpinningSetPoint * ratio,
    hardwareCommand.rightSprocketWheelSpinningSetPoint * ratio,
    hardwareCommand.leftSprocketWheelSpinningSetPoint * ratio,
    hardwareCommand.rightSprocketWheelSpinningSetPoint * ratio};
}

//-----------------------------------------------------------------------------
HardwareState2TD toHardwareState2TD(
  const double & sprocketWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const SimulationState2THD & simulationState)
{
  return {toHardwareSprocketSpinMotion(
      sprocketWheelRadius,
      idlerWheelRadius,
      trackThickness,
      simulationState.leftSprocketWheelSpinningMotion,
      simulationState.frontLeftIdlerWheelSpinningMotion,
      simulationState.rearLeftIdlerWheelSpinningMotion),
    toHardwareSprocketSpinMotion(
      sprocketWheelRadius,
      idlerWheelRadius,
      trackThickness,
      simulationState.rightSprocketWheelSpinningMotion,
      simulationState.frontRightIdlerWheelSpinningMotion,
      simulationState.rearRightIdlerWheelSpinningMotion)};
}

}  //  namespace core
}  //  namespace romea
