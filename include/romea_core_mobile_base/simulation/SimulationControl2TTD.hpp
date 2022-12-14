#ifndef ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL2TTD_HPP_
#define ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL2TTD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControl2TD.hpp"

namespace romea {


struct SimulationCommand2TTD
{
  RotationalMotionCommand leftSprocketWheelSpinningSetPoint;
  RotationalMotionCommand rightSprocketWheelSpinningSetPoint;

  RotationalMotionCommand leftIdlerWheelSpinningSetPoint;
  RotationalMotionCommand rightIdlerWheelSpinningSetPoint;

  RotationalMotionCommand frontLeftRollerWheelSpinningSetPoint;
  RotationalMotionCommand frontRightRollerWheelSpinningSetPoint;
  RotationalMotionCommand rearLeftRollerWheelSpinningSetPoint;
  RotationalMotionCommand rearRightRollerWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream &os, const SimulationCommand2TTD & state);

struct SimulationState2TTD
{
  RotationalMotionState leftSprocketWheelSpinningMotion;
  RotationalMotionState rightSprocketWheelSpinningMotion;

  RotationalMotionState leftIdlerWheelSpinningMotion;
  RotationalMotionState rightIdlerWheelSpinningMotion;

  RotationalMotionState frontLeftRollerWheelSpinningMotion;
  RotationalMotionState frontRightRollerWheelSpinningMotion;
  RotationalMotionState rearLeftRollerWheelSpinningMotion;
  RotationalMotionState rearRightRollerWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream &os, const SimulationState2TTD & state);

SimulationCommand2TTD toSimulationCommand2TTD(const double & sprocketWheelRadius,
                                              const double & rollerWheelRadius,
                                              const double & idlerWheelRadius,
                                              const double & trackThickness,
                                              const HardwareCommand2TD &hardwareCommand);

HardwareState2TD toHardwareState2TTD(const double & sprocketWheelRadius,
                                     const double & rollerWheelRadius,
                                     const double & trackThickness,
                                     const SimulationState2TTD & simulationState);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL2TTD_HPP_
