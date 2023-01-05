// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROLCOMMON_HPP_
#define ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROLCOMMON_HPP_

#include <iostream>
#include <string>

namespace romea
{

using SteeringAngleCommand = double;
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

std::ostream & operator<<(std::ostream & s, const RotationalMotionState & state);
std::ostream & operator<<(std::ostream & s, const LinearMotionState & state);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__HARDWARE__HARDWARECONTROLCOMMON_HPP_
