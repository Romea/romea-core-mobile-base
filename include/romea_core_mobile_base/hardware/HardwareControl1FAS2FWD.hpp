#ifndef romea_HardwareControl1FAS2FWD_hpp
#define romea_HardwareControl1FAS2FWD_hpp

#include "HardwareControlCommon.hpp"

namespace romea {


struct HardwareState1FAS2FWD
{
  SteeringAngleState frontAxleSteeringAngle;
  RotationalMotionState frontLeftWheelSpinMotion;
  RotationalMotionState frontRightWheelSpinMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState1FAS2FWD & state);


struct HardwareCommand1FAS2FWD
{
  SteeringAngleCommand frontAxleSteeringAngle;
  RotationalMotionCommand frontLeftWheelSetPoint;
  RotationalMotionCommand frontRightWheelSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand1FAS2FWD & command);


}//end romea
#endif
