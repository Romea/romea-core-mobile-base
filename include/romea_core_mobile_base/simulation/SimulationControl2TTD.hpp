#ifndef romea_SimulationHardwareControl2TTD_hpp
#define romea_SimulationHardwareControl2TTD_hpp

#include "romea_core_mobile_base/hardware/HardwareControl2TD.hpp"

namespace romea {


struct SimulationCommand2TTD
{
  RotationalMotionCommand leftSprocketWheelSetPoint;
  RotationalMotionCommand rightSprocketWheelSetPoint;

  RotationalMotionCommand leftIdlerWheelSetPoint;
  RotationalMotionCommand rightIdlerWheelSetPoint;

  RotationalMotionCommand frontLeftRollerWheelSetPoint;
  RotationalMotionCommand frontRightRollerWheelSetPoint;
  RotationalMotionCommand rearLeftRollerWheelSetPoint;
  RotationalMotionCommand rearRightRollerWheelSetPoint;
};

struct SimulationState2TTD
{
  RotationalMotionState leftSprocketWheelSpinMotion;
  RotationalMotionState rightSprocketWheelSpinMotion;

  RotationalMotionState leftIdlerWheelSpinMotion;
  RotationalMotionState rightIdlerWheelSpinMotion;

  RotationalMotionState frontLeftRollerWheelSpinMotion;
  RotationalMotionState frontRightRollerWheelSpinMotion;
  RotationalMotionState rearLeftRollerWheelSpinMotion;
  RotationalMotionState rearRightRollerWheelSpinMotion;
};

SimulationCommand2TTD toSimulationCommand2TTD(const double & sprocketWheelRadius,
                                              const double & rollerWheelRadius,
                                              const double & idlerWheelRadius,
                                              const HardwareCommand2TD &hardwareCommand);

HardwareState2TD toHardwareState2TTD(const double & sprocketWheelRadius,
                                     const double & rollerWheelRadius,
                                     const SimulationState2TTD & simulationState);


}//end romea
#endif
