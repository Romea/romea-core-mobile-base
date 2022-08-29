#include "romea_core_mobile_base/hardware/HardwareControl1FAS2RWD.hpp"

namespace romea {

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand1FAS2RWD & command)
{
  os << " Hardware1FAS2RWD command : "<< std::endl;
  os << " front axle steering angle : " << command.frontAxleSteeringAngle << std::endl;
  os << " rear left wheel setpoint : " << command.rearLeftWheelSetPoint << std::endl;
  os << " rear right wheel setpoint : " <<command.rearRightWheelSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState1FAS2RWD & state)
{
  os << " Hardware1FAS2RWD state : "<< std::endl;
  os << " front axle steering angle : " << state.frontAxleSteeringAngle << std::endl;
  os << " rear left wheel spin motion : " << state.rearLeftWheelSpinMotion << std::endl;
  os << " rear right wheel spin motion : " << state.rearRightWheelSpinMotion ;
  return os;
}

}//end romea
