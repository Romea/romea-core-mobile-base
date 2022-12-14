#ifndef ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2AS4WD_HPP_
#define ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2AS4WD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand2AS4WD
{
  SteeringAngleCommand frontAxleSteeringAngle;
  SteeringAngleCommand rearAxleSteeringAngle;

  RotationalMotionCommand frontLeftWheelSpinningSetPoint;
  RotationalMotionCommand frontRightWheelSpinningSetPoint;
  RotationalMotionCommand rearLeftWheelSpinningSetPoint;
  RotationalMotionCommand rearRightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand2AS4WD & command);

struct HardwareState2AS4WD
{
  SteeringAngleState frontAxleSteeringAngle;
  SteeringAngleState rearAxleSteeringAngle;

  RotationalMotionState frontLeftWheelSpinningMotion;
  RotationalMotionState frontRightWheelSpinningMotion;
  RotationalMotionState rearLeftWheelSpinningMotion;
  RotationalMotionState rearRightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState2AS4WD & state);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2AS4WD_HPP_ 
