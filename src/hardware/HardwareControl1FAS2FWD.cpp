// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_mobile_base/hardware/HardwareControl1FAS2FWD.hpp"

namespace romea
{

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const HardwareCommand1FAS2FWD & command)
{
  os << " Hardware1FAS2RWD command : " << std::endl;
  os << " front axle steering angle : ";
  os << command.frontAxleSteeringAngle << std::endl;
  os << " front left wheel spinning setpoint : ";
  os << command.frontLeftWheelSpinningSetPoint <<std::endl;
  os << " front right wheel spinning setpoint : ";
  os << command.frontRightWheelSpinningSetPoint;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const HardwareState1FAS2FWD & state)
{
  os << " Hardware1FAS2RWD state : " << std::endl;
  os << " front axle steering angle : ";
  os << state.frontAxleSteeringAngle << std::endl;
  os << " front left wheel spinning motion: ";
  os << state.frontLeftWheelSpinningMotion << std::endl;
  os << " front right wheel spinning motion : ";
  os << state.frontRightWheelSpinningMotion;
  return os;
}

}  // namespace romea
