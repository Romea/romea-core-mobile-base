#include "romea_core_mobile_base/hardware/HardwareControl1FAS2FWD.hpp"

namespace romea {

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand1FAS2FWD & command)
{
  os << " Hardware1FAS2RWD command : " << std::endl;
  os << " front axle steering angle : " << command.frontAxleSteeringAngle << std::endl;
  os << " front left wheel setpoint : " << command.frontLeftWheelSetPoint << std::endl;
  os << " front right wheel setpoint : " << command.frontRightWheelSetPoint;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState1FAS2FWD & state)
{
  os << " Hardware1FAS2RWD state : "<< std::endl;
  os << " front axle steering angle : " << state.frontAxleSteeringAngle << std::endl;
  os << " front left wheel spin motion: " << state.frontLeftWheelSpinMotion << std::endl;
  os << " front right wheel spin motion : " << state.frontRightWheelSpinMotion ;
  return os;
}

}//end romea
