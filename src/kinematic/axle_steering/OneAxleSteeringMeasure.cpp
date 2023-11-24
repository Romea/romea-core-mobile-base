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


#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringMeasure.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
OneAxleSteeringMeasure::OneAxleSteeringMeasure()
: covariance(Eigen::Matrix2d::Zero())
{
}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(
  const OneAxleSteeringMeasure & measure,
  const double & frontWheelBase,
  const double & rearWheelBase)
{
  double wheelbase = frontWheelBase + rearWheelBase;
  double tanSteeringAngle = std::tan(measure.steeringAngle);
  double instantaneousCurvature = tanSteeringAngle / wheelbase;
  double tanBeta = instantaneousCurvature * rearWheelBase;
  double alpha = (1 + tanSteeringAngle * tanSteeringAngle) / wheelbase;
  double squareGamma = (1 + tanBeta * tanBeta);
  double gamma = std::sqrt(squareGamma);

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(4, 2);
  J(0, 0) = 1;
  J(1, 0) = tanBeta;
  J(1, 1) = measure.longitudinalSpeed * alpha * rearWheelBase;
  J(2, 0) = instantaneousCurvature;
  J(2, 1) = measure.longitudinalSpeed * alpha;
  J(3, 1) = alpha / squareGamma * (gamma - instantaneousCurvature * rearWheelBase / gamma);

  KinematicMeasure convertedMeasure;
  convertedMeasure.longitudinalSpeed = measure.longitudinalSpeed;
  convertedMeasure.lateralSpeed = measure.longitudinalSpeed * tanBeta;
  convertedMeasure.angularSpeed = instantaneousCurvature * measure.longitudinalSpeed;
  convertedMeasure.instantaneousCurvature = instantaneousCurvature / gamma;
  convertedMeasure.covariance = J * measure.covariance * J.transpose();
  return convertedMeasure;
}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(
  const OneAxleSteeringMeasure & measure,
  const OneAxleSteeringKinematic::Parameters & parameters)
{
  return toKinematicMeasure(measure, parameters.frontWheelBase, parameters.rearWheelBase);
}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(
  const OneAxleSteeringMeasure & measure,
  const TwoWheelSteeringKinematic::Parameters & parameters)
{
  return toKinematicMeasure(measure, parameters.frontWheelBase, parameters.rearWheelBase);
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const OneAxleSteeringMeasure & measure)
{
  os << " OneAxleSteeringMeasure measure" << std::endl;
  os << " measured linear speed  " << measure.longitudinalSpeed << std::endl;
  os << " measured front steering angle " << measure.steeringAngle << std::endl;
  os << " measured covariance matrix " << std::endl;
  os << measure.covariance;

  return os;
}

}  // namespace core
}  // namespace romea
