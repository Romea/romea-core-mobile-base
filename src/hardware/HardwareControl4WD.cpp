#include "romea_core_mobile_base/hardware/HardwareControl4WD.hpp"

namespace romea {

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand4WD & command)
{
  os << " Hardware4WD command : " <<std::endl;
  os << " front left wheel spinning setpoint : " << command.frontLeftWheelSpinningSetPoint << std::endl;
  os << " front right wheel spinning setpoint : " << command.frontRightWheelSpinningSetPoint  << std::endl;
  os << " rear left wheel spinning setpoint : " << command.rearLeftWheelSpinningSetPoint  << std::endl;
  os << " rear right wheel spinning setpoint : " << command.rearRightWheelSpinningSetPoint  ;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState4WD & state)
{
  os << " Hardware4WD state : " << std::endl;
  os << " front left wheel spinning motion : " << state.frontLeftWheelSpinningMotion << std::endl;
  os << " front right wheel spinning motion : " << state.frontRightWheelSpinningMotion << std::endl;
  os << " rear left wheel spinning motion : " << state.rearLeftWheelSpinningMotion << std::endl;
  os << " rear right wheel spinning motion : " << state.rearRightWheelSpinningMotion ;
  return os;
}

}//end romea
