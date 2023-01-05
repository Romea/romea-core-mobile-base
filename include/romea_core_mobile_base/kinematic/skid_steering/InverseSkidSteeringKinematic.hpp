// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__INVERSESKIDSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__INVERSESKIDSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringMeasure.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2TD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2WD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame4WD.hpp"

namespace romea
{

void inverseKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const OdometryFrame2TD & odometryFrame,
  SkidSteeringMeasure & skidSteeringMeasure);

void inverseKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const OdometryFrame2WD & odometryFrame,
  SkidSteeringMeasure & skidSteeringMeasure);

void inverseKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const OdometryFrame4WD & odometryFrame,
  SkidSteeringMeasure & skidSteeringMeasure);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__INVERSESKIDSTEERINGKINEMATIC_HPP_
