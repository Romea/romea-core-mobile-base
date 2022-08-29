#ifndef romea_HardwareControl1FAS2RWD_hpp
#define romea_HardwareControl1FAS2RWD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareState1FAS2RWD
{
  SteeringAngleState frontAxleSteeringAngle;
  RotationalMotionState rearLeftWheelSpinMotion;
  RotationalMotionState rearRightWheelSpinMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState1FAS2RWD & state);

struct HardwareCommand1FAS2RWD
{
  SteeringAngleCommand frontAxleSteeringAngle;
  RotationalMotionCommand rearLeftWheelSetPoint;
  RotationalMotionCommand rearRightWheelSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand1FAS2RWD & command);


}//end romea
#endif
