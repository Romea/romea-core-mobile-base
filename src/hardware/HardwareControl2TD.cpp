#include "romea_core_mobile_base/hardware/HardwareControl2TD.hpp"

namespace romea {

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand2TD & command)
{
  os << " Hardware2WD command coucoun : " <<std::endl;
  os << " left  sprocket wheel spinning setpoint : " << command.leftSprocketWheelSpinningSetPoint << std::endl;
  os << " right sprocket wheel spinning setpoint : " << command.rightSprocketWheelSpinningSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState2TD & state)
{
  os << " Hardware2WD state : " << std::endl;
  os << " left  sprocket wheel rotational motion : " << state.leftSprocketWheelSpinningMotion << std::endl;
  os << " right sprocket wheel rotational motion : " << state.rightSprocketWheelSpinningMotion << std::endl;
  return os;
}

}  // namespace romea
