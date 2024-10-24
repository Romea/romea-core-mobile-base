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


// romea core
#include <romea_core_common/math/Algorithm.hpp>

// std
#include <algorithm>
#include <limits>

// local
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringMeasure.hpp"

namespace
{
const double EPSILON = 0.0000001;
}

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
SkidSteeringMeasure::SkidSteeringMeasure()
: covariance(Eigen::Matrix2d::Zero())
{
}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(const SkidSteeringMeasure & measure)
{
  KinematicMeasure convertedMeasure;
  convertedMeasure.longitudinalSpeed = measure.longitudinalSpeed;
  convertedMeasure.angularSpeed = measure.angularSpeed;

  if (std::abs(measure.angularSpeed) < EPSILON) {
    convertedMeasure.instantaneousCurvature = 0;
  } else if (std::abs(measure.longitudinalSpeed) > EPSILON) {
    convertedMeasure.instantaneousCurvature = measure.angularSpeed / measure.longitudinalSpeed;
  } else {
    convertedMeasure.instantaneousCurvature =
      sign(measure.angularSpeed) * std::numeric_limits<double>::max();
  }


  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(4, 2);
  J(0, 0) = 1;
  J(2, 1) = 1;
  J(3, 1) = measure.angularSpeed;

  if (std::abs(measure.longitudinalSpeed) < EPSILON) {
    J(
      3,
      0) = -measure.angularSpeed /
      (std::pow(sign(measure.longitudinalSpeed) * EPSILON, 2));
  }

  convertedMeasure.covariance = J * measure.covariance * J.transpose();
  return convertedMeasure;
}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(
  const SkidSteeringMeasure & command,
  const SkidSteeringKinematic::Parameters & /*parameters*/)
{
  return toKinematicMeasure(command);
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const SkidSteeringMeasure & measure)
{
  os << " SkidSteering measure " << std::endl;
  os << " measured linear speed " << measure.longitudinalSpeed << std::endl;
  os << " measured angular speed " << measure.angularSpeed << std::endl;
  os << " measured covariance matrix " << std::endl;
  os << measure.covariance;

  return os;
}

}  // namespace core
}  // namespace romea


// old codes
// //-----------------------------------------------------------------------------
// SkidSteeringMeasure toSkidSteeringMeasure(const KinematicMeasure & measure)
// {
//   assert(measure.beta < std::numeric_limits<double>::epsilon());
//   SkidSteeringMeasure convertedMeasure;
//   convertedMeasure.longitudinalSpeed = measure.speed;
//   convertedMeasure.angularSpeed = measure.angularSpeed;
//   convertedMeasure.covariance(0, 0) = measure.covariance(0, 0);
//   convertedMeasure.covariance(0, 1) = measure.covariance(0, 2);
//   convertedMeasure.covariance(1, 0) = measure.covariance(2, 0);
//   convertedMeasure.covariance(1, 1) = measure.covariance(2, 2);
//   return convertedMeasure;
// }

// //-----------------------------------------------------------------------------
// SkidSteeringMeasure toSkidSteeringCommand(
//   const KinematicMeasure & measure,
//   const Kinematic & /*kinematic*/)
// {
//   return toSkidSteeringMeasure(measure);
// }
