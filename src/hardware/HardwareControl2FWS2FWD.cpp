// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_mobile_base/hardware/HardwareControl2FWS2FWD.hpp"

namespace romea {


//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand2FWS2FWD & command)
{
  os << " Hardware2FWS2FWD command : " <<std::endl;
  os << " front left wheel steering angle : ";
  os << command.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : ";
  os << command.frontRightWheelSteeringAngle << std::endl;
  os << " front left wheel spinning setpoint ";
  os << command.frontLeftWheelSpinningSetPoint << std::endl;
  os << " front right wheel spinning setpoint ";
  os << command.frontRightWheelSpinningSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState2FWS2FWD & state)
{
  os << " Hardware2FWS2FWD state : " << std::endl;
  os << " front left wheel steering angle : ";
  os << state.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : ";
  os << state.frontRightWheelSteeringAngle << std::endl;
  os << " front left wheel rotational motion : ";
  os << state.frontLeftWheelSpinningMotion << std::endl;
  os << " front right wheel rotational motion : ";
  os << state.frontRightWheelSpinningMotion << std::endl;
  return os;
}

}  // namespace romea
