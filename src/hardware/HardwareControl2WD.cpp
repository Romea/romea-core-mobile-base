#include "romea_core_mobile_base/hardware/HardwareControl2WD.hpp"

namespace romea {

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand2WD & command)
{
  os << " Hardware2WD command : " <<std::endl;
  os << " left wheel setpoint : " << command.leftWheelSetPoint << std::endl;
  os << " right wheel setpoint : " << command.rightWheelSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState2WD & state)
{
  os << " Hardware2WD state : " << std::endl;
  os << " left wheel rotational motion : " << state.leftWheelSpinMotion << std::endl;
  os << " right wheel rotational motion : " << state.rightWheelSpinMotion << std::endl;
  return os;
}

}//end romea
