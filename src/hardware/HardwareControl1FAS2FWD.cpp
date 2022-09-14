#include "romea_core_mobile_base/hardware/HardwareControl1FAS2FWD.hpp"

namespace romea {

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareCommand1FAS2FWD & command)
{
  os << " Hardware1FAS2RWD command : " << std::endl;
  os << " front axle steering angle : " << command.frontAxleSteeringAngle << std::endl;
  os << " front left wheel spinning setpoint : " << command.frontLeftWheelSpinningSetPoint << std::endl;
  os << " front right wheel spinning setpoint : " << command.frontRightWheelSpinningSetPoint;
  return os;
}

//------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const HardwareState1FAS2FWD & state)
{
  os << " Hardware1FAS2RWD state : "<< std::endl;
  os << " front axle steering angle : " << state.frontAxleSteeringAngle << std::endl;
  os << " front left wheel spinning motion: " << state.frontLeftWheelSpinningMotion << std::endl;
  os << " front right wheel spinning motion : " << state.frontRightWheelSpinningMotion ;
  return os;
}

}//end romea
