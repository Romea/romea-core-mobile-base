#ifndef ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2AS2FWD_HPP_
#define ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2AS2FWD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

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

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2AS2FWD_HPP_
