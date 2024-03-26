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


// romea
#include "romea_core_mobile_base/kinematic/axle_steering/InverseOneAxleSteeringKinematic.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
void inverseKinematic(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OdometryFrame1FAS2FWD & odometryFrame,
  OneAxleSteeringMeasure & oneAxleSteeringMeasure)
{
  const double & wheelSpeedVariance = parameters.wheelLinearSpeedVariance;
  const double & steeringAngleVariance = parameters.steeringAngleVariance;

  const double halfWheelTrack = parameters.frontWheelTrack / 2;
  const double wheelBase = parameters.frontWheelBase + parameters.rearWheelBase;
  const double hubCarrierOffset = parameters.rearHubCarrierOffset;

  const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelLinearSpeed;
  const double & frontRightWheelSpeed = odometryFrame.frontRightWheelLinearSpeed;
  const double & frontSteeringAngle = odometryFrame.frontAxleSteeringAngle;

  double tanFrontSteeringAngle = std::tan(frontSteeringAngle);
  double instantaneousCurvature = tanFrontSteeringAngle / wheelBase;
  double instantaneousCurvatureHalfTrack_ = instantaneousCurvature * halfWheelTrack;

  double alphaLeft = 1 - instantaneousCurvatureHalfTrack_;
  double alphaRight = 1 + instantaneousCurvatureHalfTrack_;
  double squareTanFrontSteeringAngle = tanFrontSteeringAngle * tanFrontSteeringAngle;
  double betaLeft = std::sqrt(alphaLeft * alphaLeft + squareTanFrontSteeringAngle);
  double betaRight = std::sqrt(alphaRight * alphaRight + squareTanFrontSteeringAngle);
  double gammaLeft = betaLeft - tanFrontSteeringAngle * hubCarrierOffset / wheelBase;
  double gammaRight = betaRight + tanFrontSteeringAngle * hubCarrierOffset / wheelBase;


  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = wheelSpeedVariance;
  covariance(1, 1) = wheelSpeedVariance;
  covariance(2, 2) = steeringAngleVariance;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2, 3);
  J(0, 0) = 0.5 / gammaLeft;
  J(0, 1) = 0.5 / gammaRight;
  J(0, 2) +=
    (2 * (alphaLeft * halfWheelTrack / wheelBase + tanFrontSteeringAngle) / betaLeft -
    hubCarrierOffset / wheelBase) / (gammaLeft * gammaLeft);
  J(0, 2) +=
    (2 * (alphaRight * halfWheelTrack / wheelBase + tanFrontSteeringAngle) / betaRight -
    hubCarrierOffset / wheelBase) / (gammaRight * gammaRight);
  J(0, 2) *= 0.5 * (1 + tanFrontSteeringAngle * tanFrontSteeringAngle);
  J(1, 2) = 1;

  oneAxleSteeringMeasure.steeringAngle = frontSteeringAngle;
  oneAxleSteeringMeasure.longitudinalSpeed = 0.5 *
    (frontLeftWheelSpeed / gammaLeft + frontRightWheelSpeed / gammaRight);
  oneAxleSteeringMeasure.covariance = J * covariance * J.transpose();
}

//-----------------------------------------------------------------------------
void inverseKinematic(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OdometryFrame1FAS2RWD & odometryFrame,
  OneAxleSteeringMeasure & oneAxleSteeringMeasure)
{
  const double & wheelSpeedVariance = parameters.wheelLinearSpeedVariance;
  const double & steeringAngleVariance = parameters.steeringAngleVariance;

  const double & steeringAngle = odometryFrame.frontAxleSteeringAngle;
  const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelLinearSpeed;
  const double & rearRightWheelSpeed = odometryFrame.rearRightWheelLinearSpeed;

  oneAxleSteeringMeasure.steeringAngle = steeringAngle;
  oneAxleSteeringMeasure.longitudinalSpeed = 0.5 * (rearLeftWheelSpeed + rearRightWheelSpeed);
  oneAxleSteeringMeasure.covariance(0, 0) = 0.5 * wheelSpeedVariance;
  oneAxleSteeringMeasure.covariance(1, 1) = steeringAngleVariance;
}


//-----------------------------------------------------------------------------
void inverseKinematic(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OdometryFrame1FAS4WD & odometryFrame,
  OneAxleSteeringMeasure & oneAxleSteeringMeasure)
{
  const double & wheelSpeedVariance = parameters.wheelLinearSpeedVariance;
  const double & steeringAngleVariance = parameters.steeringAngleVariance;

  const double frontHalfWheelTrack = parameters.frontWheelTrack / 2;
  const double wheelBase = parameters.frontWheelBase + parameters.rearWheelBase;
  const double hubCarrierOffset = parameters.rearHubCarrierOffset;

  const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelLinearSpeed;
  const double & frontRightWheelSpeed = odometryFrame.frontRightWheelLinearSpeed;
  const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelLinearSpeed;
  const double & rearRightWheelSpeed = odometryFrame.rearRightWheelLinearSpeed;
  const double & frontSteeringAngle = odometryFrame.frontAxleSteeringAngle;

  double tanFrontSteeringAngle = std::tan(frontSteeringAngle);
  double instantaneousCurvature = tanFrontSteeringAngle / wheelBase;
  double instantaneousCurvatureHalfTrack_ = instantaneousCurvature * frontHalfWheelTrack;

  double alphaLeft = 1 - instantaneousCurvatureHalfTrack_;
  double alphaRight = 1 + instantaneousCurvatureHalfTrack_;
  double squareTanFrontSteeringAngle = tanFrontSteeringAngle * tanFrontSteeringAngle;
  double betaLeft = std::sqrt(alphaLeft * alphaLeft + squareTanFrontSteeringAngle);
  double betaRight = std::sqrt(alphaRight * alphaRight + squareTanFrontSteeringAngle);
  double gammaLeft = betaLeft - tanFrontSteeringAngle * hubCarrierOffset / wheelBase;
  double gammaRight = betaRight + tanFrontSteeringAngle * hubCarrierOffset / wheelBase;


  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(5, 5);
  covariance(0, 0) = wheelSpeedVariance;
  covariance(1, 1) = wheelSpeedVariance;
  covariance(2, 2) = wheelSpeedVariance;
  covariance(3, 3) = wheelSpeedVariance;
  covariance(2, 2) = steeringAngleVariance;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2, 5);
  J(0, 0) = 0.25 / gammaLeft;
  J(0, 1) = 0.25 / gammaRight;
  J(0, 2) = 0.25;
  J(0, 3) = 0.25;
  J(0, 4) +=
    (2 * (alphaLeft * frontHalfWheelTrack / wheelBase + tanFrontSteeringAngle) / betaLeft -
    hubCarrierOffset / wheelBase) / (gammaLeft * gammaLeft);
  J(0, 4) +=
    (2 * (alphaRight * frontHalfWheelTrack / wheelBase + tanFrontSteeringAngle) / betaRight -
    hubCarrierOffset / wheelBase) / (gammaRight * gammaRight);
  J(0, 4) *= 0.5 * (1 + tanFrontSteeringAngle * tanFrontSteeringAngle);
  J(1, 4) = 1;

  oneAxleSteeringMeasure.steeringAngle = frontSteeringAngle;
  oneAxleSteeringMeasure.longitudinalSpeed =
    0.25 * (frontLeftWheelSpeed / gammaLeft + frontRightWheelSpeed / gammaRight) +
    0.25 * (rearLeftWheelSpeed + rearRightWheelSpeed);
  oneAxleSteeringMeasure.covariance = J * covariance * J.transpose();
}


}  // namespace core
}  // namespace romea
