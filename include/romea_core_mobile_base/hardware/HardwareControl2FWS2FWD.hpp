#ifndef romea_HardwareControl2FWS2FWD_hpp
#define romea_HardwareControl2FWS2FWD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand2FWS2FWD
{
  SteeringAngleCommand frontLeftWheelSteeringAngle;
  SteeringAngleCommand frontRightWheelSteeringAngle;

  RotationalMotionCommand frontLeftWheelSetPoint;
  RotationalMotionCommand frontRightWheelSetPoint;
};

std::ostream & operator<<(std::ostream &s, const HardwareCommand2FWS2FWD &frame);

struct HardwareState2FWS2FWD
{
  SteeringAngleState frontLeftWheelSteeringAngle;
  SteeringAngleState frontRightWheelSteeringAngle;

  RotationalMotionState frontLeftWheelSpinMotion;
  RotationalMotionState frontRightWheelSpinMotion;
};

std::ostream & operator<<(std::ostream &s, const HardwareState2FWS2FWD &frame);

}//end romea
#endif
