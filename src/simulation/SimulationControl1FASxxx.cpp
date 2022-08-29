#include "romea_core_mobile_base/simulation/SimulationControl1FASxxx.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const SimulationCommand1FASxxx & command)
{
  os << " Hardware1FASxxx command : "<< std::endl;
  os << " front axle steering angle : " << command.frontAxleSteeringAngle << std::endl;
  os << " front axle steering angle : " << command.frontLeftWheelSteeringAngle << std::endl;
  os << " front axle steering angle : " << command.frontRightWheelSteeringAngle << std::endl;
  os << " front left wheel setpoint : " << command.frontLeftWheelSetPoint << std::endl;
  os << " front right wheel setpoint : " <<command.frontRightWheelSetPoint << std::endl;
  os << " rear left wheel setpoint : " << command.rearLeftWheelSetPoint << std::endl;
  os << " rear right wheel setpoint : " <<command.rearRightWheelSetPoint << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const SimulationState1FASxxx & state)
{
  os << " Hardware1FASxxx state : "<< std::endl;
  os << " front axle steering angle : " << state.frontAxleSteeringAngle << std::endl;
  os << " front left wheel steering angle : " << state.frontLeftWheelSteeringAngle << std::endl;
  os << " front right wheel steering angle : " << state.frontRightWheelSteeringAngle << std::endl;
  os << " front left wheel spin motion: " << state.frontLeftWheelSpinMotion << std::endl;
  os << " front right wheel spin motion : " << state.frontRightWheelSpinMotion << std::endl;
  os << " rear left wheel spin motion: " << state.rearLeftWheelSpinMotion << std::endl;
  os << " rear right wheel spin motion : " << state.rearRightWheelSpinMotion ;
  return os;
}


}
