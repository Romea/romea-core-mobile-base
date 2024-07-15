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
#include <ostream>
#include <cmath>

// romea
#include "romea_core_mobile_base/simulation/SimulationControl2TD.hpp"


namespace
{

romea::core::RotationalMotionState toHardwareSprocketSpinMotion(
  const double & sprocketWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const romea::core::RotationalMotionState & sprocketWheelSpinningMotion,
  const romea::core::RotationalMotionState & idlerWheelSpinningMotion)
{
  const double sprocketWheelVirtualRadius = sprocketWheelRadius + trackThickness;
  const double idlerWheelVirtualRadius = idlerWheelRadius + trackThickness;
  const double ratio = idlerWheelVirtualRadius / sprocketWheelVirtualRadius;

  const double sprocketWheelLinearSpeed = sprocketWheelVirtualRadius *
    sprocketWheelSpinningMotion.velocity;
  const double idlerWheelLinearSpeed = idlerWheelVirtualRadius *
    idlerWheelSpinningMotion.velocity;

  romea::core::RotationalMotionState output;
  output.position = sprocketWheelSpinningMotion.position;

  if (std::signbit(sprocketWheelLinearSpeed) != std::signbit(idlerWheelLinearSpeed)) {
    output.velocity = 0;
    output.torque = 0;
  } else if (std::abs(sprocketWheelLinearSpeed) < std::abs(idlerWheelLinearSpeed)) {
    output.velocity = sprocketWheelSpinningMotion.velocity;
    output.torque = sprocketWheelSpinningMotion.torque;
  } else {
    output.velocity = idlerWheelSpinningMotion.velocity * ratio;
    output.torque = idlerWheelSpinningMotion.torque * ratio;
  }
  return output;
}

}  // namespace

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const SimulationCommand2TD & command)
{
  os << " Simulation2TD command : " << std::endl;
  os << " left sprocket wheel spinning setpoint : " <<
    command.leftSprocketWheelSpinningSetPoint << std::endl;
  os << " right sprocket wheel spinning setpoint : " <<
    command.rightSprocketWheelSpinningSetPoint << std::endl;
  os << " left idler wheel spinning setpoint : " <<
    command.leftIdlerWheelSpinningSetPoint << std::endl;
  os << " right idler wheel spinning setpoint : " <<
    command.rightIdlerWheelSpinningSetPoint << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const SimulationState2TD & state)
{
  os << " Simulation2TD state : " << std::endl;
  os << " left sprocket wheel spinning motion: " <<
    state.leftSprocketWheelSpinningMotion << std::endl;
  os << " right sprocket wheel spinning motion : " <<
    state.rightSprocketWheelSpinningMotion << std::endl;
  os << " left idler wheel spinning motion: " <<
    state.leftIdlerWheelSpinningMotion << std::endl;
  os << " right idler wheel spinning motion : " << state.rightIdlerWheelSpinningMotion;
  return os;
}


//-----------------------------------------------------------------------------
SimulationCommand2TD toSimulationCommand2TD(
  const double & sprocketWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const HardwareCommand2TD & hardwareCommand)
{
  const double ratio = (sprocketWheelRadius + trackThickness) / (idlerWheelRadius + trackThickness);

  return {hardwareCommand.leftSprocketWheelSpinningSetPoint,
    hardwareCommand.rightSprocketWheelSpinningSetPoint,
    hardwareCommand.leftSprocketWheelSpinningSetPoint * ratio,
    hardwareCommand.rightSprocketWheelSpinningSetPoint * ratio};
}

//-----------------------------------------------------------------------------
SimulationState2TD  toSimulationState2TD(
  const double & sprocketWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const HardwareState2TD & hardwareState)
{
  HardwareCommand2TD fakeHardwareCommand = {
    hardwareState.leftSprocketWheelSpinningMotion.velocity,
    hardwareState.rightSprocketWheelSpinningMotion.velocity
  };


  SimulationCommand2TD fakeSimulationCommand = toSimulationCommand2TD(
    sprocketWheelRadius,
    idlerWheelRadius,
    trackThickness,
    fakeHardwareCommand);

  SimulationState2TD simulationState;
  simulationState.leftSprocketWheelSpinningMotion =
    hardwareState.leftSprocketWheelSpinningMotion;
  simulationState.rightSprocketWheelSpinningMotion =
    hardwareState.rightSprocketWheelSpinningMotion;
  simulationState.leftIdlerWheelSpinningMotion.velocity =
    fakeSimulationCommand.leftIdlerWheelSpinningSetPoint;
  simulationState.rightIdlerWheelSpinningMotion.velocity =
    fakeSimulationCommand.rightIdlerWheelSpinningSetPoint;

  return simulationState;
}

//-----------------------------------------------------------------------------
HardwareState2TD toHardwareState2TD(
  const double & sprocketWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const SimulationState2TD & simulationState)
{
  return {toHardwareSprocketSpinMotion(
      sprocketWheelRadius,
      idlerWheelRadius,
      trackThickness,
      simulationState.leftSprocketWheelSpinningMotion,
      simulationState.leftIdlerWheelSpinningMotion),
    toHardwareSprocketSpinMotion(
      sprocketWheelRadius,
      idlerWheelRadius,
      trackThickness,
      simulationState.rightSprocketWheelSpinningMotion,
      simulationState.rightIdlerWheelSpinningMotion)};
}

}  // namespace core
}  // namespace romea
