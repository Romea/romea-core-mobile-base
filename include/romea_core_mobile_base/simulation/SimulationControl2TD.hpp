// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2TD_HPP_
#define ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2TD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControl2TD.hpp"

namespace romea
{


struct SimulationCommand2TD
{
  RotationalMotionCommand leftSprocketWheelSpinningSetPoint;
  RotationalMotionCommand rightSprocketWheelSpinningSetPoint;

  RotationalMotionCommand leftIdlerWheelSpinningSetPoint;
  RotationalMotionCommand rightIdlerWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream & os, const SimulationCommand2TD & state);

struct SimulationState2TD
{
  RotationalMotionState leftSprocketWheelSpinningMotion;
  RotationalMotionState rightSprocketWheelSpinningMotion;

  RotationalMotionState leftIdlerWheelSpinningMotion;
  RotationalMotionState rightIdlerWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream & os, const SimulationState2TD & state);

SimulationCommand2TD toSimulationCommand2TD(
  const double & sprocketWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const HardwareCommand2TD & hardwareCommand);

HardwareState2TD toHardwareState2TD(
  const double & sprocketWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const SimulationState2TD & simulationState);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2TD_HPP_
