#ifndef romea_HardwareControl4WD_hpp
#define romea_HardwareControl4WD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareState4WD
{
  RotationalMotionState frontLeftWheelSpinMotion;
  RotationalMotionState frontRightWheelSpinMotion;
  RotationalMotionState rearLeftWheelSpinMotion;
  RotationalMotionState rearRightWheelSpinMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState4WD & state);

struct HardwareCommand4WD
{
  RotationalMotionCommand frontLeftWheelSetPoint;
  RotationalMotionCommand frontRightWheelSetPoint;
  RotationalMotionCommand rearLeftWheelSetPoint;
  RotationalMotionCommand rearRightWheelSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand4WD & command);



}//end romea
#endif
