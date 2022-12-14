#ifndef ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2FWS2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2FWS2RWD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

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

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2FWS2RWD_HPP_
