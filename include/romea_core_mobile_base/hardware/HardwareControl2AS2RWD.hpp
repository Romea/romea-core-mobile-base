#ifndef romea_HardwareControl2AS2RWD_hpp
#define romea_HardwareControl2AS2RWD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand2AS2RWD
{
  SteeringAngleCommand frontAxleSteeringAngle;
  SteeringAngleCommand rearAxleSteeringAngle;

  RotationalMotionCommand rearLeftWheelSpinningSetPoint;
  RotationalMotionCommand rearRightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand2AS2RWD & command);

struct HardwareState2AS2RWD
{
  SteeringAngleState frontAxleSteeringAngle;
  SteeringAngleState rearAxleSteeringAngle;

  RotationalMotionState rearLeftWheelSpinningMotion;
  RotationalMotionState rearRightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState2AS2RWD & state);

}//end romea
#endif
