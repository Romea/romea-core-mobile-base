#ifndef romea_HardwareControlCommon_hpp
#define romea_HardwareControlCommon_hpp

#include <iostream>

namespace romea {

using SteeringAngleCommand =double;
using SteeringAngleState = double;


struct RotationalMotionState
{
  RotationalMotionState();
  double position;
  double velocity;
  double torque;
};

using RotationalMotionCommand = double;

enum class RotationalMotionControlType
{
  VELOCITY,
  TORQUE
};

std::string toCommandType(RotationalMotionControlType type);

struct LinearMotionState
{
  LinearMotionState();
  double position;
  double velocity;
  double force;
};

using LinearMotionCommand = double;

enum class LinearMotionControlType
{
  VELOCITY,
  FORCE
};

std::ostream & operator<<(std::ostream &s, const RotationalMotionState & state);
std::ostream & operator<<(std::ostream &s, const LinearMotionState & state);

//template <typename HardwareCommandType>
//struct HardwareCommand
//{
//  RotationalMotionCommandType wheelsSetPointType
//  HardwareCommandType command;
//}


}//end romea
#endif
