#ifndef romea_HardwareControl4WS4WD_hpp
#define romea_HardwareControl4WS4WD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {


struct HardwareCommand4WS4WD
{
  SteeringAngleCommand frontLeftWheelSteeringAngle;
  SteeringAngleCommand frontRightWheelSteeringAngle;
  SteeringAngleCommand rearLeftWheelSteeringAngle;
  SteeringAngleCommand rearRightWheelSteeringAngle;

  RotationalMotionCommand frontLeftWheelSetPoint;
  RotationalMotionCommand frontRightWheelSetPoint;
  RotationalMotionCommand rearLeftWheelSetPoint;
  RotationalMotionCommand rearRightWheelSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand4WS4WD & command);

struct HardwareState4WS4WD
{
  SteeringAngleState frontLeftWheelSteeringAngle;
  SteeringAngleState frontRightWheelSteeringAngle;
  SteeringAngleState rearLeftWheelSteeringAngle;
  SteeringAngleState rearRightWheelSteeringAngle;

  RotationalMotionState frontLeftWheelSpinMotion;
  RotationalMotionState frontRightWheelSpinMotion;
  RotationalMotionState rearLeftWheelSpinMotion;
  RotationalMotionState rearRightWheelSpinMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState4WS4WD & state);

}//end romea
#endif
