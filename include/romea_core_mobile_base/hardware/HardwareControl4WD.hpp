#ifndef romea_HardwareControl4WD_hpp
#define romea_HardwareControl4WD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareState4WD
{
  RotationalMotionState frontLeftWheelSpinningMotion;
  RotationalMotionState frontRightWheelSpinningMotion;
  RotationalMotionState rearLeftWheelSpinningMotion;
  RotationalMotionState rearRightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState4WD & state);

struct HardwareCommand4WD
{
  RotationalMotionCommand frontLeftWheelSpinningSetPoint;
  RotationalMotionCommand frontRightWheelSpinningSetPoint;
  RotationalMotionCommand rearLeftWheelSpinningSetPoint;
  RotationalMotionCommand rearRightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand4WD & command);



}//end romea
#endif
