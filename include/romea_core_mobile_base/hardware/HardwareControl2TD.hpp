#ifndef ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2TD_HPP_
#define ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2TD_HPP_

#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

namespace romea {

struct HardwareCommand2TD
{
  RotationalMotionCommand leftSprocketWheelSpinningSetPoint;
  RotationalMotionCommand rightSprocketWheelSpinningSetPoint;
};

std::ostream & operator<<(std::ostream &os, const HardwareCommand2TD & command);

struct HardwareState2TD
{
  RotationalMotionState leftSprocketWheelSpinningMotion;
  RotationalMotionState rightSprocketWheelSpinningMotion;
};

std::ostream & operator<<(std::ostream &os, const HardwareState2TD & state);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_HARDWARE_HARDWARECONTROL2TD_HPP_ 
