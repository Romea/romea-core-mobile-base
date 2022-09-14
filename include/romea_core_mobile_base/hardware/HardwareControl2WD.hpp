#ifndef romea_HardwareControl2WD_hpp
#define romea_HardwareControl2WD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand2WD
{
  RotationalMotionCommand leftWheelSpinningSetPoint;
  RotationalMotionCommand rightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand2WD & command);

struct HardwareState2WD
{
  RotationalMotionState leftWheelSpinningMotion;
  RotationalMotionState rightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState2WD & state);

}//end romea
#endif
