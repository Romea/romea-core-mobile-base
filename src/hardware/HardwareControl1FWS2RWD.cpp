#include "romea_core_mobile_base/hardware/HardwareControl1FWS2RWD.hpp"

namespace romea {

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand1FWS2RWD & command)
{  os << " Hardware1FWS2RWD command : "<< std::endl;
  os << " front wheel steering angle : " << command.frontWheelSteeringAngle << std::endl;
  os << " rear left wheel setpoint : " << command.rearLeftWheelSetPoint << std::endl;
  os << " rear right wheel setpoint : " << command.rearRightWheelSetPoint ;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState1FWS2RWD & state)
{
  os << " Hardware1FWS2RWD state : "<< std::endl;
  os << " front wheel steering angle : " << state.frontWheelSteeringAngle << std::endl;
  os << " rear left wheel spin motion : " << state.rearLeftWheelSpinMotion << std::endl;
  os << " rear right wheel spin motion: " << state.rearRightWheelSpinMotion ;
  return os;
}

}//end romea
