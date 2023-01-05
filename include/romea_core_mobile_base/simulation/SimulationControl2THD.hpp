// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL2THD_HPP_
#define ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL2THD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControl2TD.hpp"

namespace romea
{


struct SimulationCommand2THD
{
  RotationalMotionCommand leftSprocketWheelSpinningSetPoint;
  RotationalMotionCommand rightSprocketWheelSpinningSetPoint;

  RotationalMotionCommand frontLeftIdlerWheelSpinningSetPoint;
  RotationalMotionCommand frontRightIdlerWheelSpinningSetPoint;
  RotationalMotionCommand rearLeftIdlerWheelSpinningSetPoint;
  RotationalMotionCommand rearRightIdlerWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream & os, const SimulationCommand2THD & state);

struct SimulationState2THD
{
  RotationalMotionState leftSprocketWheelSpinningMotion;
  RotationalMotionState rightSprocketWheelSpinningMotion;

  RotationalMotionState frontLeftIdlerWheelSpinningMotion;
  RotationalMotionState frontRightIdlerWheelSpinningMotion;
  RotationalMotionState rearLeftIdlerWheelSpinningMotion;
  RotationalMotionState rearRightIdlerWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream & os, const SimulationCommand2THD & state);

SimulationCommand2THD toSimulationCommand2THD(
  const double & sprocketWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const HardwareCommand2TD & hardwareCommand);

HardwareState2TD toHardwareState2TD(
  const double & sprocketWheelRadius,
  const double & idlerWheelRadius,
  const double & trackThickness,
  const SimulationState2THD & simulationState);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2THD_HPP_
