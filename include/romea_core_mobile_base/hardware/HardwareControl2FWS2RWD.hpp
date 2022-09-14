#ifndef romea_HardwareControl2FWS2RWD_hpp
#define romea_HardwareControl2FWS2RWD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand2FWS2RWD
{
  SteeringAngleCommand frontLeftWheelSteeringAngle;
  SteeringAngleCommand frontRightWheelSteeringAngle;

  RotationalMotionCommand rearLeftWheelSpinningSetPoint;
  RotationalMotionCommand rearRightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream &s, const HardwareCommand2FWS2RWD &frame);

struct HardwareState2FWS2RWD
{
  SteeringAngleState frontLeftWheelSteeringAngle;
  SteeringAngleState frontRightWheelSteeringAngle;

  RotationalMotionState rearLeftWheelSpinningMotion;
  RotationalMotionState rearRightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream &s, const HardwareState2FWS2RWD &frame);

}//end romea
#endif
