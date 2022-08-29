#include "romea_core_mobile_base/hardware/HardwareControl2AS4WD.hpp"

namespace romea {


//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand2AS4WD & command)
{
  os << " Hardware2AS4WD command : "<< std::endl;
  os << " front axle steering angle : " << command.frontAxleSteeringAngle << std::endl;
  os << " rear axle steering angle : " << command.rearAxleSteeringAngle << std::endl;
  os << " front left wheel setpoint : " << command.frontLeftWheelSetPoint << std::endl;
  os << " front right wheel setpoint : " << command.frontRightWheelSetPoint << std::endl;
  os << " rear left wheel setpoint : " << command.rearLeftWheelSetPoint << std::endl;
  os << " rear right wheel setpoint : " << command.rearRightWheelSetPoint ;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState2AS4WD & state)
{
  os << " front axle steering angle : " << state.frontAxleSteeringAngle << std::endl;
  os << " rear axle steering angle : " << state.rearAxleSteeringAngle << std::endl;
  os << " front left wheel spin motion : " << state.frontLeftWheelSpinMotion << std::endl;
  os << " front right wheel spin motion : " << state.frontRightWheelSpinMotion << std::endl;
  os << " rear left wheel spin motion : " << state.rearLeftWheelSpinMotion << std::endl;
  os << " rear right wheel spin motion : " << state.rearRightWheelSpinMotion ;
  return os;
}

}//end romea
