// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__TWOAXLESTEERINGMEASURE_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__TWOAXLESTEERINGMEASURE_HPP_

// Eigen
#include <Eigen/Core>

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/KinematicMeasure.hpp"


namespace romea
{


struct TwoAxleSteeringMeasure : public TwoAxleSteeringCommand
{
  TwoAxleSteeringMeasure();
  Eigen::Matrix3d covariance;
};


KinematicMeasure toKinematicMeasure(
  const TwoAxleSteeringMeasure & measure,
  const double & frontWheelBase,
  const double & rearWheelBase);

KinematicMeasure toKinematicMeasure(
  const TwoAxleSteeringMeasure & measure,
  const TwoAxleSteeringKinematic::Parameters & parameters);

KinematicMeasure toKinematicMeasure(
  const TwoAxleSteeringMeasure & measure,
  const FourWheelSteeringKinematic::Parameters & parameters);

std::ostream & operator<<(std::ostream & os, const TwoAxleSteeringMeasure & command);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__TWOAXLESTEERINGMEASURE_HPP_
