#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

#include <limits>

namespace romea {


//-----------------------------------------------------------------------------
std::string toCommandType(RotationalMotionControlType type)
{
  switch(type)
  {
  case RotationalMotionControlType::VELOCITY :
    return "angular velocity";
  case RotationalMotionControlType::TORQUE :
    return "torque";
  }
}

//-----------------------------------------------------------------------------
RotationalMotionState::RotationalMotionState():
  position(std::numeric_limits<double>::quiet_NaN()),
  velocity(std::numeric_limits<double>::quiet_NaN()),
  torque(std::numeric_limits<double>::quiet_NaN())
{
}


//-----------------------------------------------------------------------------
std::string toCommandType(LinearMotionControlType type)
{
  switch(type)
  {
  case LinearMotionControlType::VELOCITY :
    return "linear velocity";
  case LinearMotionControlType::FORCE :
    return "force";
  }
}

//-----------------------------------------------------------------------------
LinearMotionState::LinearMotionState():
  position(std::numeric_limits<double>::quiet_NaN()),
  velocity(std::numeric_limits<double>::quiet_NaN()),
  force(std::numeric_limits<double>::quiet_NaN())
{
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream &os, const RotationalMotionState & state)
{
  os << " position ";
  os << state.position;
  os << ", velocity ";
  os << state.velocity;
  os << ", torque ";
  os << state.torque;
  return os;
}

//-----------------------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const LinearMotionState & state)
{
  os << " position ";
  os << state.position;
  os << ", velocity ";
  os << state.velocity;
  os << ", force ";
  os << state.force;
  return os;
}


}//end romea
