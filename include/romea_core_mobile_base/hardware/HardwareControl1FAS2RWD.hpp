// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROL1FAS2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROL1FAS2RWD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

namespace romea
{

struct HardwareState1FAS2RWD
{
  SteeringAngleState frontAxleSteeringAngle;
  RotationalMotionState rearLeftWheelSpinningMotion;
  RotationalMotionState rearRightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream & os, const HardwareState1FAS2RWD & state);

struct HardwareCommand1FAS2RWD
{
  SteeringAngleCommand frontAxleSteeringAngle;
  RotationalMotionCommand rearLeftWheelSpinningSetPoint;
  RotationalMotionCommand rearRightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream & os, const HardwareCommand1FAS2RWD & command);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROL1FAS2RWD_HPP_
