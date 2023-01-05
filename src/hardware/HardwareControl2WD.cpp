// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_mobile_base/hardware/HardwareControl2WD.hpp"

namespace romea
{

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const HardwareCommand2WD & command)
{
  os << " Hardware2WD command : " << std::endl;
  os << " left wheel spinning setpoint : ";
  os << command.leftWheelSpinningSetPoint << std::endl;
  os << " right wheel spinning setpoint : ";
  os << command.rightWheelSpinningSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const HardwareState2WD & state)
{
  os << " Hardware2WD state : " << std::endl;
  os << " left wheel rotational motion : ";
  os << state.leftWheelSpinningMotion << std::endl;
  os << " right wheel rotational motion : ";
  os << state.rightWheelSpinningMotion << std::endl;
  return os;
}

}  // namespace romea
