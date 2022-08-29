#ifndef romea_HardwareControl2AS4WD_hpp
#define romea_HardwareControl2AS4WD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand2AS4WD
{
  SteeringAngleCommand frontAxleSteeringAngle;
  SteeringAngleCommand rearAxleSteeringAngle;

  RotationalMotionCommand frontLeftWheelSetPoint;
  RotationalMotionCommand frontRightWheelSetPoint;
  RotationalMotionCommand rearLeftWheelSetPoint;
  RotationalMotionCommand rearRightWheelSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand2AS4WD & command);

struct HardwareState2AS4WD
{
  SteeringAngleState frontAxleSteeringAngle;
  SteeringAngleState rearAxleSteeringAngle;

  RotationalMotionState frontLeftWheelSpinMotion;
  RotationalMotionState frontRightWheelSpinMotion;
  RotationalMotionState rearLeftWheelSpinMotion;
  RotationalMotionState rearRightWheelSpinMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState2AS4WD & state);

}//end romea
#endif
