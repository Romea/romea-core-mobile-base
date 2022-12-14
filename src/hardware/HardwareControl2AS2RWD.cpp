#include "romea_core_mobile_base/hardware/HardwareControl2AS2RWD.hpp"

namespace romea {


//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand2AS2RWD & command)
{
  os << " Hardware2AS2RWD command : "<< std::endl;
  os << " front axle steering angle : " << command.frontAxleSteeringAngle << std::endl;
  os << " rear axle steering angle : " << command.rearAxleSteeringAngle << std::endl;
  os << " rear left wheel spinning setpoint : " << command.rearLeftWheelSpinningSetPoint << std::endl;
  os << " rear right wheel spinning setpoint : " << command.rearRightWheelSpinningSetPoint ;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState2AS2RWD & state)
{
  os << " Hardware2AS2RWD command : "<< std::endl;
  os << " front axle steering angle : " << state.frontAxleSteeringAngle << std::endl;
  os << " rear axle steering angle : " << state.rearAxleSteeringAngle << std::endl;
  os << " rear left wheel spinning motion : " << state.rearLeftWheelSpinningMotion << std::endl;
  os << " rear right wheel spinning motion : " << state.rearRightWheelSpinningMotion ;
  return os;
}

}  // namespace romea
