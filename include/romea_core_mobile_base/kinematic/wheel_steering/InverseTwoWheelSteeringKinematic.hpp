// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__INVERSETWOWHEELSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__INVERSETWOWHEELSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/KinematicMeasure.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringMeasure.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2FWS2FWD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2FWS2RWD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2FWS4WD.hpp"

namespace romea
{

void inverseKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OdometryFrame2FWS2FWD & odometryFrame,
  OneAxleSteeringMeasure & oneAxleSteeringMeasure);

void inverseKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OdometryFrame2FWS2RWD & odometryFrame,
  OneAxleSteeringMeasure & oneAxleSteeringMeasure);

void inverseKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OdometryFrame2FWS4WD & odometryFrame,
  OneAxleSteeringMeasure & oneAxleSteeringMeasure);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__INVERSETWOWHEELSTEERINGKINEMATIC_HPP_
