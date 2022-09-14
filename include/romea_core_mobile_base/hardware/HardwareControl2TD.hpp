#ifndef romea_HardwareControl2TD_hpp
#define romea_HardwareControl2TD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand2TD
{
  RotationalMotionCommand leftSprocketWheelSpinningSetPoint;
  RotationalMotionCommand rightSprocketWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand2TD & command);

struct HardwareState2TD
{
  RotationalMotionState leftSprocketWheelSpinningMotion;
  RotationalMotionState rightSprocketWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState2TD & state);

}//end romea
#endif
