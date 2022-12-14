#include "romea_core_mobile_base/hardware/HardwareControl2FWS2FWD.hpp"

namespace romea {


//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand2FWS2FWD & command)
{
  os << " Hardware2FWS2FWD command : " <<std::endl;
  os << " front left wheel steering angle : " << command.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : " << command.frontRightWheelSteeringAngle << std::endl;
  os << " front left wheel spinning setpoint "<< command.frontLeftWheelSpinningSetPoint << std::endl;
  os << " front right wheel spinning setpoint "<< command.frontRightWheelSpinningSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState2FWS2FWD & state)
{
  os << " Hardware2FWS2FWD state : " << std::endl;
  os << " front left wheel steering angle : "   << state.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : " << state.frontRightWheelSteeringAngle << std::endl;
  os << " front left wheel rotational motion : " << state.frontLeftWheelSpinningMotion << std::endl;
  os << " front right wheel rotational motion : " << state.frontRightWheelSpinningMotion << std::endl;
  return os;
}

}  // namespace romea
