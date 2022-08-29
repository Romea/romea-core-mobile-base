#ifndef romea_HardwareControl1FWS2RWD_hpp
#define romea_HardwareControl1FWS2RWD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand1FWS2RWD
{
  SteeringAngleCommand frontWheelSteeringAngle;
  RotationalMotionCommand rearLeftWheelSetPoint;
  RotationalMotionCommand rearRightWheelSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand1FWS2RWD & command);

struct HardwareState1FWS2RWD
{
  SteeringAngleState frontWheelSteeringAngle;
  RotationalMotionState rearLeftWheelSpinMotion;
  RotationalMotionState rearRightWheelSpinMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState1FWS2RWD & state);

}//end romea
#endif
