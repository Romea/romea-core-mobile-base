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


#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringMeasure.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
TwoAxleSteeringMeasure::TwoAxleSteeringMeasure()
: covariance(Eigen::Matrix3d::Zero())
{
}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(
  const TwoAxleSteeringMeasure & measure,
  const double & frontWheelBase,
  const double & rearWheelBase)
{
  double wheelbase = frontWheelBase + rearWheelBase;
  double tanFrontSteeringAngle = std::tan(measure.frontSteeringAngle);
  double tanRearSteeringAngle = std::tan(measure.rearSteeringAngle);
  double instantaneousCurvature = (tanFrontSteeringAngle - tanRearSteeringAngle) / wheelbase;
  double tanBeta = (tanFrontSteeringAngle * rearWheelBase +
    tanRearSteeringAngle * frontWheelBase) / wheelbase;

  double frontAlpha = (1 + tanFrontSteeringAngle * tanFrontSteeringAngle) / wheelbase;
  double rearAlpha = (1 + tanRearSteeringAngle * tanRearSteeringAngle) / wheelbase;
  double squareGamma = (1 + tanBeta * tanBeta);
  double gamma = std::sqrt(squareGamma);

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(4, 3);
  J(0, 0) = 1;
  J(1, 0) = tanBeta;
  J(1, 1) = measure.longitudinalSpeed * frontAlpha * rearWheelBase;
  J(1, 2) = measure.longitudinalSpeed * rearAlpha * frontWheelBase;
  J(2, 0) = instantaneousCurvature;
  J(2, 1) = measure.longitudinalSpeed * frontAlpha;
  J(2, 1) = measure.longitudinalSpeed * rearAlpha;
  J(3, 1) = frontAlpha / squareGamma * (gamma - instantaneousCurvature * rearWheelBase / gamma);
  J(3, 2) = rearAlpha / squareGamma * (gamma - instantaneousCurvature * frontWheelBase / gamma);

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
  const TwoAxleSteeringMeasure & measure,
  const TwoAxleSteeringKinematic::Parameters & parameters)
{
  return toKinematicMeasure(
    measure,
    parameters.frontWheelBase,
    parameters.rearWheelBase);
}

//-----------------------------------------------------------------------------
KinematicMeasure toKinematicMeasure(
  const TwoAxleSteeringMeasure & measure,
  const FourWheelSteeringKinematic::Parameters & parameters)
{
  return toKinematicMeasure(
    measure,
    parameters.frontWheelBase,
    parameters.rearWheelBase);
}

//-----------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const TwoAxleSteeringMeasure & measure)
{
  os << " TwoAxleSteering Measure   " << std::endl;
  os << " measured linear speed  " << measure.longitudinalSpeed << std::endl;
  os << " measured front steering angle " << measure.frontSteeringAngle << std::endl;
  os << " measured rear steering angle " << measure.rearSteeringAngle << std::endl;
  os << " covariance matrix" << std::endl;
  os << measure.covariance << std::endl;
  return os;
}

}  // namespace core
}  // namespace romea


// old codes
////-----------------------------------------------------------------------------
// TwoAxleSteeringMeasure toTwoAxleSteeringMeasure(const KinematicMeasure & measure,
//                                                const double & frontWheelBase,
//                                                const double & rearWheelBase)
//{

//  double sinBeta = std::sin(measure.beta);
//  double cosBeta = std::cos(measure.beta);
//  double alphaf= measure.instantaneousCurvature*frontWheelBase;
//  double alphar= -measure.instantaneousCurvature*rearWheelBase;
//  double deltaf = alphaf+sinBeta;
//  double deltar = alphar+sinBeta;

//  double gammaf = deltaf*deltaf+cosBeta*cosBeta;
//  double gammar = deltar*deltar+cosBeta*cosBeta;

//  double linearSpeed = measure.speed*std::cos(measure.beta);
//  double frontSteeringAngle = std::atan2(deltaf,cosBeta);
//  double rearSteeringAngle = std::atan2(deltar,cosBeta);


//  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3,4);
//  J(0,0)= std::cos(measure.beta);
//  J(0,1)= -measure.speed*std::sin(measure.beta);
//  J(1,1)=  (1 +sinBeta*alphaf)/gammaf;
//  J(1,3)=  frontWheelBase*cosBeta/gammaf;
//  J(2,1)=  (1 +sinBeta*alphar)/gammar;
//  J(2,3)= -rearWheelBase*cosBeta/gammar;

//  TwoAxleSteeringMeasure convertedMeasure;
//  convertedMeasure.longitudinalSpeed=linearSpeed;
//  convertedMeasure.frontSteeringAngle=frontSteeringAngle;
//  convertedMeasure.rearSteeringAngle=rearSteeringAngle;
//  convertedMeasure.covariance = J*measure.covariance*J.transpose();
//  return convertedMeasure;
//}


////-----------------------------------------------------------------------------
// TwoAxleSteeringMeasure toTwoAxleSteeringMeasure(const KinematicMeasure & measure,
//                                                const Kinematic & kinematic)
//{
//  return toTwoAxleSteeringMeasure(measure,
//                                  kinematic.getWheelBase("front_wheelbase"),
//                                  kinematic.getWheelBase("rear_wheelbase"));
//}
