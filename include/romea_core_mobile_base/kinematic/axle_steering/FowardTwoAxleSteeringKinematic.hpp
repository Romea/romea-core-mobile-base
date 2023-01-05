// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__FOWARDTWOAXLESTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__FOWARDTWOAXLESTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2AS4WD.hpp"

namespace romea
{

void forwardKinematic(
  const TwoAxleSteeringKinematic::Parameters & parameters,
  const TwoAxleSteeringCommand & commandFrame,
  OdometryFrame2AS4WD & odometryCommandFrame);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__FOWARDTWOAXLESTEERINGKINEMATIC_HPP_
