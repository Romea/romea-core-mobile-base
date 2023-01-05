// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__INVERSEONEAXLESTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__INVERSEONEAXLESTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringMeasure.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS2FWD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS2RWD.hpp"

namespace romea
{

void inverseKinematic(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OdometryFrame1FAS2FWD & odometryFrame,
  OneAxleSteeringMeasure & oneAxleSteeringMeasure);

void inverseKinematic(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OdometryFrame1FAS2RWD & odometryFrame,
  OneAxleSteeringMeasure & oneAxleSteeringMeasure);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__INVERSEONEAXLESTEERINGKINEMATIC_HPP_
