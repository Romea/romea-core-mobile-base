#ifndef romea_HardwareControl2TD_hpp
#define romea_HardwareControl2TD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand2TD
{
  RotationalMotionCommand leftSprocketWheelSetPoint;
  RotationalMotionCommand rightSprocketWheelSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand2TD & command);

struct HardwareState2TD
{
  RotationalMotionState leftSprocketWheelSpinMotion;
  RotationalMotionState rightSprocketWheelSpinMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState2TD & state);

}//end romea
#endif
