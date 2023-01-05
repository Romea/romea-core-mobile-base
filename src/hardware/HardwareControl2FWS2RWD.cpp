// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_mobile_base/hardware/HardwareControl2FWS2RWD.hpp"

namespace romea
{

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const HardwareCommand2FWS2RWD & command)
{
  os << " Hardware2FWS2RWD command : " << std::endl;
  os << " front left wheel steering angle : ";
  os << command.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : ";
  os << command.frontRightWheelSteeringAngle << std::endl;
  os << " rear left wheel spinning setpoint : ";
  os << command.rearLeftWheelSpinningSetPoint << std::endl;
  os << " rear right wheel spinning setpoint : ";
  os << command.rearRightWheelSpinningSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const HardwareState2FWS2RWD & state)
{
  os << " Hardware2FWS2RWD state : " << std::endl;
  os << " front left wheel steering angle : ";
  os << state.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : ";
  os << state.frontRightWheelSteeringAngle << std::endl;
  os << " rear left wheel rotational  : ";
  os << state.rearLeftWheelSpinningMotion << std::endl;
  os << " rear right wheel rotational motion : ";
  os << state.rearRightWheelSpinningMotion << std::endl;
  return os;
}

}  // namespace romea
