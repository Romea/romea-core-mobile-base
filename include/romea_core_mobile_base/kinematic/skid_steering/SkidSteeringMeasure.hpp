// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__SKIDSTEERINGMEASURE_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__SKIDSTEERINGMEASURE_HPP_

// romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/KinematicMeasure.hpp"

namespace romea
{

struct SkidSteeringMeasure : public SkidSteeringCommand
{
  SkidSteeringMeasure();
  Eigen::Matrix2d covariance;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::ostream & operator<<(std::ostream & os, const SkidSteeringMeasure & measure);

KinematicMeasure toKinematicMeasure(const SkidSteeringMeasure & measure);

KinematicMeasure toKinematicMeasure(
  const SkidSteeringMeasure & measure,
  const SkidSteeringKinematic::Parameters & parameters);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__SKIDSTEERINGMEASURE_HPP_
