#ifndef romea_HardwareControl2AS2FWD_hpp
#define romea_HardwareControl2AS2FWD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand2AS2FWD
{
  SteeringAngleCommand frontAxleSteeringAngle;
  SteeringAngleCommand rearAxleSteeringAngle;

  RotationalMotionCommand frontLeftWheelSpinningSetPoint;
  RotationalMotionCommand frontRightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand2AS2FWD & command);

struct HardwareState2AS2FWD
{
  SteeringAngleState frontAxleSteeringAngle;
  SteeringAngleState rearAxleSteeringAngle;

  RotationalMotionState frontLeftWheelSpinningMotion;
  RotationalMotionState frontRightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState2AS2FWD & state);

}//end romea
#endif
