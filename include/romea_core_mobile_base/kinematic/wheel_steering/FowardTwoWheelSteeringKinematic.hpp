// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__FOWARDTWOWHEELSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__FOWARDTWOWHEELSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2FWS2FWD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2FWS2RWD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2FWS4WD.hpp"

namespace romea
{

void forwardKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & commandFrame,
  OdometryFrame2FWS2FWD & commandOdometryFrame);

void forwardKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & commandFrame,
  OdometryFrame2FWS2RWD & commandOdometryFrame);

void forwardKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & commandFrame,
  OdometryFrame2FWS4WD & commandOdometryFrame);

}  // namespace romea

#endif ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__FOWARDTWOWHEELSTEERINGKINEMATIC_HPP_
