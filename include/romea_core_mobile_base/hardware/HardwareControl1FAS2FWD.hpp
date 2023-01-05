// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROL1FAS2FWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROL1FAS2FWD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

namespace romea
{


struct HardwareState1FAS2FWD
{
  SteeringAngleState frontAxleSteeringAngle;
  RotationalMotionState frontLeftWheelSpinningMotion;
  RotationalMotionState frontRightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream & os, const HardwareState1FAS2FWD & state);


struct HardwareCommand1FAS2FWD
{
  SteeringAngleCommand frontAxleSteeringAngle;
  RotationalMotionCommand frontLeftWheelSpinningSetPoint;
  RotationalMotionCommand frontRightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream & os, const HardwareCommand1FAS2FWD & command);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROL1FAS2FWD_HPP_
