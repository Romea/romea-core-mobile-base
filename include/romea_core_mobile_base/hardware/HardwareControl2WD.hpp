// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROL2WD_HPP_
#define ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROL2WD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand2WD
{
  RotationalMotionCommand leftWheelSpinningSetPoint;
  RotationalMotionCommand rightWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand2WD & command);

struct HardwareState2WD
{
  RotationalMotionState leftWheelSpinningMotion;
  RotationalMotionState rightWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState2WD & state);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROL2WD_HPP_
