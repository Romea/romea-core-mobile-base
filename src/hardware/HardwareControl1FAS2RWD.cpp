#include "romea_core_mobile_base/hardware/HardwareControl1FAS2RWD.hpp"

namespace romea {

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand1FAS2RWD & command)
{
  os << " Hardware1FAS2RWD command : "<< std::endl;
  os << " front axle steering angle : " << command.frontAxleSteeringAngle << std::endl;
  os << " rear left wheel spinning setpoint : " << command.rearLeftWheelSpinningSetPoint << std::endl;
  os << " rear right wheel spinning setpoint : " <<command.rearRightWheelSpinningSetPoint << std::endl;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState1FAS2RWD & state)
{
  os << " Hardware1FAS2RWD state : "<< std::endl;
  os << " front axle steering angle : " << state.frontAxleSteeringAngle << std::endl;
  os << " rear left wheel spinning motion : " << state.rearLeftWheelSpinningMotion << std::endl;
  os << " rear right wheel spinning motion : " << state.rearRightWheelSpinningMotion ;
  return os;
}

}  // namespace romea
