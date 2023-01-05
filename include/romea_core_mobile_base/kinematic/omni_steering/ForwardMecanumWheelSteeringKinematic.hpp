// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__FORWARDMECANUMWHEELSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__FORWARDMECANUMWHEELSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/MecanumWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2WD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame4WD.hpp"

namespace romea
{

void forwardKinematic(
  const MecanumWheelSteeringKinematic::Parameters & parameters,
  const OmniSteeringCommand & commandFrame,
  OdometryFrame4WD & odometryFrame);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__FORWARDMECANUMWHEELSTEERINGKINEMATIC_HPP_
