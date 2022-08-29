#ifndef romea_HardwareControl1FAS2RWD_hpp
#define romea_HardwareControl1FAS2RWD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareState1FAS4WD
{
  SteeringAngleState frontAxleSteeringAngle;
  RotationalMotionState frontLeftWheelSpinMotion;
  RotationalMotionState frontRightWheelSpinMotion;
  RotationalMotionState rearLeftWheelSpinMotion;
  RotationalMotionState rearRightWheelSpinMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState1FAS4WD & state);

struct HardwareCommand1FAS4WD
{
  SteeringAngleCommand frontAxleSteeringAngle;
  RotationalMotionCommand frontLeftWheelSetPoint;
  RotationalMotionCommand frontRightWheelSetPoint;
  RotationalMotionCommand rearLeftWheelSetPoint;
  RotationalMotionCommand rearRightWheelSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareState1FAS4WD & command);


}//end romea
#endif
