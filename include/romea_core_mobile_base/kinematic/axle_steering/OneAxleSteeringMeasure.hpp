// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE__STEERING_ONEAXLESTEERINGMEASURE_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE__STEERING_ONEAXLESTEERINGMEASURE_HPP_

// Eigen
#include <Eigen/Core>

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/KinematicMeasure.hpp"


namespace romea
{
namespace core
{

struct OneAxleSteeringMeasure : OneAxleSteeringCommand
{
  OneAxleSteeringMeasure();

  Eigen::Matrix2d covariance;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::ostream & operator<<(std::ostream & os, const OneAxleSteeringMeasure & measure);

KinematicMeasure toKinematicMeasure(
  const OneAxleSteeringMeasure & measure,
  const double & frontWheelBase,
  const double & rearWheelBase);

KinematicMeasure toKinematicMeasure(
  const OneAxleSteeringMeasure & measure,
  const OneAxleSteeringKinematic::Parameters & parameters);

KinematicMeasure toKinematicMeasure(
  const OneAxleSteeringMeasure & measure,
  const TwoWheelSteeringKinematic::Parameters & parameters);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGMEASURE_HPP_
