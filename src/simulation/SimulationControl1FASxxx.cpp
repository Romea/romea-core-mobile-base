// std
#include <iostream>

// romea
#include "romea_core_mobile_base/simulation/SimulationControl1FASxxx.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const SimulationCommand1FASxxx & command)
{
  os << " Simulation1FASxxx command : "<< std::endl;
  os << " front axle steering angle : "
     << command.frontAxleSteeringAngle << std::endl;
  os << " front left wheel steering angle : "
     << command.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : "
     << command.frontRightWheelSteeringAngle << std::endl;
  os << " front left wheel spinning setpoint : "
     << command.frontLeftWheelSpinningSetPoint << std::endl;
  os << " front right wheel spinning setpoint : "
     << command.frontRightWheelSpinningSetPoint << std::endl;
  os << " rear left wheel spinning setpoint : "
     << command.rearLeftWheelSpinningSetPoint << std::endl;
  os << " rear right wheel spinning setpoint : "
     << command.rearRightWheelSpinningSetPoint << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const SimulationState1FASxxx & state)
{
  os << " Simulation1FASxxx state : "<< std::endl;
  os << " front axle steering angle : "
     << state.frontAxleSteeringAngle << std::endl;
  os << " front left wheel steering angle : "
     << state.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : "
     << state.frontRightWheelSteeringAngle << std::endl;
  os << " front left wheel spinning motion: "
     << state.frontLeftWheelSpinningMotion << std::endl;
  os << " front right wheel spinning motion : "
     << state.frontRightWheelSpinningMotion << std::endl;
  os << " rear left wheel spinning motion: "
     << state.rearLeftWheelSpinningMotion << std::endl;
  os << " rear right wheel spinning motion : "
     << state.rearRightWheelSpinningMotion ;
  return os;
}

}  // namespace romea
