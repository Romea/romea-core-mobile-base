// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__FOWARDFOURWHEELSTEERINGKINEMATIC
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__FOWARDFOURWHEELSTEERINGKINEMATIC

// romea
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame4WS4WD.hpp"

namespace romea
{

void forwardKinematic(
  const FourWheelSteeringKinematic::Parameters & kinematic,
  const TwoAxleSteeringCommand & commandFrame,
  OdometryFrame4WS4WD & commandOdometryFrame);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__FOWARDFOURWHEELSTEERINGKINEMATIC
