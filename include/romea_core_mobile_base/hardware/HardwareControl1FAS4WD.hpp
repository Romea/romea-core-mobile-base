#ifndef romea_HardwareControl1FAS4WD_hpp
#define romea_HardwareControl1FAS4WD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {

struct HardwareState1FAS4WD
{
  SteeringAngleState frontAxleSteeringAngle;
  RotationalMotionState frontLeftWheelSpinningMotion;
  RotationalMotionState frontRightWheelSpinningMotion;
  RotationalMotionState rearLeftWheelSpinningMotion;
  RotationalMotionState rearRightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState1FAS4WD & state);

struct HardwareCommand1FAS4WD
{
  SteeringAngleCommand frontAxleSteeringAngle;
  RotationalMotionCommand frontLeftWheelSpinningSetPoint;
  RotationalMotionCommand frontRightWheelSpinningSetPoint;
  RotationalMotionCommand rearLeftWheelSpinningSetPoint;
  RotationalMotionCommand rearRightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareState1FAS4WD & command);


}//end romea
#endif
