#include "romea_core_mobile_base/hardware/HardwareControl2TD.hpp"

namespace romea {

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand2TD & command)
{
  os << " Hardware2WD command : " <<std::endl;
  os << " left  sprocket wheel setpoint : " << command.leftSprocketWheelSetPoint << std::endl;
  os << " right sprocket wheel setpoint : " << command.rightSprocketWheelSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState2TD & state)
{
  os << " Hardware2WD state : " << std::endl;
  os << " left  sprocket wheel rotational motion : " << state.leftSprocketWheelSpinMotion << std::endl;
  os << " right sprocket wheel rotational motion : " << state.rightSprocketWheelSpinMotion << std::endl;
  return os;
}

}//end romea
