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
  os << " front left wheel spinning setpoint : "<< command.frontLeftWheelSpinningSetPoint << std::endl;
  os << " front right wheel spinning setpoint : " << command.frontRightWheelSpinningSetPoint << std::endl;
  os << " rear left wheel spinning setpoint : " << command.rearLeftWheelSpinningSetPoint << std::endl;
  os << " rear right wheel spinning setpoint : " << command.rearRightWheelSpinningSetPoint ;
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
  os << " front left wheel rotational motion : " << state.frontLeftWheelSpinningMotion << std::endl;
  os << " front right wheel rotational motion : " << state.frontRightWheelSpinningMotion << std::endl;
  os << " rear left wheel rotational motion : " << state.rearLeftWheelSpinningMotion << std::endl;
  os << " rear right wheel rotational motion : " << state.rearRightWheelSpinningMotion ;
  return os;
}

}  // namespace romea
