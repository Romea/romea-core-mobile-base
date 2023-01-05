// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGMEASURE_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGMEASURE_HPP_

// romea
#include "romea_core_mobile_base/kinematic/KinematicMeasure.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/MecanumWheelSteeringKinematic.hpp"

namespace romea
{

struct OmniSteeringMeasure : public OmniSteeringCommand
{
  OmniSteeringMeasure();
  Eigen::Matrix3d covariance;
};

std::ostream & operator<<(std::ostream & os, const OmniSteeringMeasure & measure);

KinematicMeasure toKinematicMeasure(const OmniSteeringMeasure & measure);

KinematicMeasure toKinematicMeasure(
  const OmniSteeringMeasure & measure,
  const MecanumWheelSteeringKinematic::Parameters & parameters);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGMEASURE_HPP_
