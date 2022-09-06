#ifndef romea_SimulationHardwareControl2TD_hpp
#define romea_SimulationHardwareControl2TD_hpp

#include "romea_core_mobile_base/hardware/HardwareControl2TD.hpp"

namespace romea {


struct SimulationCommand2TD
{
  RotationalMotionCommand leftSprocketWheelSetPoint;
  RotationalMotionCommand rightSprocketWheelSetPoint;

  RotationalMotionCommand leftIdlerWheelSetPoint;
  RotationalMotionCommand rightIdlerWheelSetPoint;
};

struct SimulationState2TD
{
  RotationalMotionState leftSprocketWheelSpinMotion;
  RotationalMotionState rightSprocketWheelSpinMotion;

  RotationalMotionState leftIdlerWheelSpinMotion;
  RotationalMotionState rightIdlerWheelSpinMotion;
};


SimulationCommand2TD toSimulationCommand2TD(const double & sprocketWheelRadius,
                                            const double & idlerWheelRadius,
                                            const double & trackThickness,
                                            const HardwareCommand2TD &hardwareCommand);

HardwareState2TD toHardwareState2TD(const double & sprocketWheelRadius,
                                    const double & idlerWheelRadius,
                                    const double & trackThickness,
                                    const SimulationState2TD & simulationState);


}//end romea
#endif
