#include "romea_core_mobile_base/hardware/HardwareControl4WS4WD.hpp"

namespace romea {

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand4WS4WD & command)
{
  os << " Hardware4WS4WD command : " <<std::endl;
  os << " front left wheel steering angle : "   << command.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : " << command.frontRightWheelSteeringAngle << std::endl;
  os << " rear left wheel steering angle : " << command.rearLeftWheelSteeringAngle << std::endl;
  os << " rear right wheel steering angle : " << command.rearRightWheelSteeringAngle << std::endl;
  os << " front left wheel setpoint : "<< command.frontLeftWheelSetPoint << std::endl;
  os << " front right wheel setpoint : " << command.frontRightWheelSetPoint << std::endl;
  os << " rear left wheel setpoint : " << command.rearLeftWheelSetPoint << std::endl;
  os << " rear right wheel setpoint : " << command.rearRightWheelSetPoint ;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState4WS4WD & state)
{
  os << " Hardware4WS4WD state : " << std::endl;
  os << " front left wheel steering angle : "   << state.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : " << state.frontRightWheelSteeringAngle << std::endl;
  os << " rear left wheel steering angle : " << state.rearLeftWheelSteeringAngle << std::endl;
  os << " rear right wheel steering angle : " << state.rearRightWheelSteeringAngle << std::endl;
  os << " front left wheel rotational motion : " << state.frontLeftWheelSpinMotion << std::endl;
  os << " front right wheel rotational motion : " << state.frontRightWheelSpinMotion << std::endl;
  os << " rear left wheel rotational motion : " << state.rearLeftWheelSpinMotion << std::endl;
  os << " rear right wheel rotational motion : " << state.rearRightWheelSpinMotion ;
  return os;
}

}//end romea
