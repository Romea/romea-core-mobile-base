#include "romea_core_mobile_base/hardware/HardwareControl2FWS2RWD.hpp"

namespace romea {

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand2FWS2RWD & command)
{
  os << " Hardware2FWS2RWD command : " <<std::endl;
  os << " front left wheel steering angle : "   << command.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : " << command.frontRightWheelSteeringAngle << std::endl;
  os << " rear left wheel setpoint : " << command.rearLeftWheelSetPoint << std::endl;
  os << " rear right wheel setpoint : " << command.rearRightWheelSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState2FWS2RWD & state)
{
  os << " Hardware2FWS2RWD state : " << std::endl;
  os << " front left wheel steering angle : "   << state.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : " << state.frontRightWheelSteeringAngle << std::endl;
  os << " rear left wheel rotational  : " << state.rearLeftWheelSpinMotion << std::endl;
  os << " rear right wheel rotational motion : " << state.rearRightWheelSpinMotion << std::endl;
  return os;
}

}//end romea
