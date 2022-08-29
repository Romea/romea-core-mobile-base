#ifndef romea_SimulationControl2THD_hpp
#define romea_SimulationControl2THD_hpp

#include "romea_core_mobile_base/hardware/HardwareControl2TD.hpp"

namespace romea {


struct SimulationCommand2THD
{
  RotationalMotionCommand leftSprocketWheelSetPoint;
  RotationalMotionCommand rightSprocketWheelSetPoint;

  RotationalMotionCommand frontLeftIdlerWheelSetPoint;
  RotationalMotionCommand frontRightIdlerWheelSetPoint;
  RotationalMotionCommand rearLeftIdlerWheelSetPoint;
  RotationalMotionCommand rearRightIdlerWheelSetPoint;

};

struct SimulationState2THD
{
  RotationalMotionState leftSprocketWheelSpinMotion;
  RotationalMotionState rightSprocketWheelSpinMotion;

  RotationalMotionState frontLeftIdlerWheelSpinMotion;
  RotationalMotionState frontRightIdlerWheelSpinMotion;
  RotationalMotionState rearLeftIdlerWheelSpinMotion;
  RotationalMotionState rearRightIdlerWheelSpinMotion;
};

SimulationCommand2THD toSimulationCommand2THD(const double & sprocketWheelRadius,
                                              const double & idlerWheelRadius,
                                              const HardwareCommand2TD &hardwareCommand);

HardwareState2TD toHardwareState2TD(const double & sprocketWheelRadius,
                                    const double & idlerWheelRadius,
                                    const SimulationState2THD & simulationState);


}//end romea
#endif
