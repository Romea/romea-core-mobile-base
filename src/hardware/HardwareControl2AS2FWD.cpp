#include "romea_core_mobile_base/hardware/HardwareControl2AS2FWD.hpp"

namespace romea {


//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand2AS2FWD & command)
{
  os << " Hardware2AS2FWD command : "<< std::endl;
  os << " front axle steering angle : " << command.frontAxleSteeringAngle << std::endl;
  os << " rear axle steering angle : " << command.rearAxleSteeringAngle << std::endl;
  os << " front left wheel spinning setpoint : " << command.frontLeftWheelSpinningSetPoint << std::endl;
  os << " front right wheel spinning setpoint : " << command.frontRightWheelSpinningSetPoint ;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState2AS2FWD & state)
{
  os << " Hardware2AS2FWD state : "<< std::endl;
  os << " front axle steering angle : " << state.frontAxleSteeringAngle << std::endl;
  os << " rear axle steering angle : " << state.rearAxleSteeringAngle << std::endl;
  os << " front left wheel spinning motion : " << state.frontLeftWheelSpinningMotion << std::endl;
  os << " front right wheel spinning motion : " << state.frontRightWheelSpinningMotion ;
  return os;
}

}  // namespace romea
