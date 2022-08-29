#include "romea_core_mobile_base/hardware/HardwareControl2FWS2FWD.hpp"

namespace romea {


//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand2FWS2FWD & command)
{
  os << " Hardware2FWS2FWD command : " <<std::endl;
  os << " front left wheel steering angle : " << command.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : " << command.frontRightWheelSteeringAngle << std::endl;
  os << " front left wheel setpoint "<< command.frontLeftWheelSetPoint << std::endl;
  os << " front right wheel setpoint "<< command.frontRightWheelSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState2FWS2FWD & state)
{
  os << " Hardware2FWS2FWD state : " << std::endl;
  os << " front left wheel steering angle : "   << state.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : " << state.frontRightWheelSteeringAngle << std::endl;
  os << " front left wheel rotational motion : " << state.frontLeftWheelSpinMotion << std::endl;
  os << " front right wheel rotational motion : " << state.frontRightWheelSpinMotion << std::endl;
  return os;
}

}//end romea
