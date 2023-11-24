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
#include "romea_core_mobile_base/kinematic/wheel_steering/InverseTwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
void inverseKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OdometryFrame2FWS2FWD & odometryFrame,
  OneAxleSteeringMeasure & oneAxleSteeringMeasure)
{
  const double & wheelSpeedVariance = parameters.wheelLinearSpeedVariance;
  const double & wheelAngleVariance = parameters.wheelSteeringAngleVariance;

  const double halfTrack = parameters.frontWheelTrack / 2;
  const double wheelBase = parameters.frontWheelBase + parameters.rearWheelBase;
  const double hubCarrierOffset = parameters.frontHubCarrierOffset;

  const double & frontLeftWheelAngle = odometryFrame.frontLeftWheelSteeringAngle;
  const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelLinearSpeed;
  double sinLeft = std::sin(frontLeftWheelAngle);
  double cosLeft = std::cos(frontLeftWheelAngle);
  double alphaLeft = cosLeft + sinLeft * halfTrack / wheelBase;
  double betaLeft = 1 - hubCarrierOffset * sinLeft / wheelBase;

  const double & frontRightWheelAngle = odometryFrame.frontRightWheelSteeringAngle;
  const double & frontRightWheelSpeed = odometryFrame.frontRightWheelLinearSpeed;
  double sinRight = std::sin(frontRightWheelAngle);
  double cosRight = std::cos(frontRightWheelAngle);
  double alphaRight = cosRight - sinRight * halfTrack / wheelBase;
  double betaRight = 1 + hubCarrierOffset * sinRight / wheelBase;

  double gamma = 1 / (1 + std::pow(0.5 * (sinLeft / alphaLeft + sinRight / alphaRight), 2));

  Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
  covariance(0, 0) = wheelSpeedVariance;
  covariance(1, 1) = wheelSpeedVariance;
  covariance(2, 2) = wheelAngleVariance;
  covariance(3, 3) = wheelAngleVariance;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2, 4);
  J(0, 0) = 0.5 * alphaLeft / betaLeft;
  J(0, 1) = 0.5 * alphaRight / betaRight;
  J(
    0,
    2) = frontLeftWheelSpeed *
    ((-sinLeft + cosLeft * halfTrack / wheelBase) - alphaLeft * hubCarrierOffset * cosLeft /
    wheelBase) / (betaLeft * betaLeft);
  J(
    0,
    3) = frontRightWheelSpeed *
    ((-sinRight - cosRight * halfTrack / wheelBase) - alphaRight * hubCarrierOffset * cosRight /
    wheelBase) / (betaRight * betaRight);
  J(1, 2) = gamma / (alphaLeft * alphaLeft);
  J(1, 3) = gamma / (alphaRight * alphaRight);


  oneAxleSteeringMeasure.steeringAngle =
    std::atan(0.5 * (sinLeft / alphaLeft + sinRight / alphaRight));

  oneAxleSteeringMeasure.longitudinalSpeed =
    0.5 *
    (frontLeftWheelSpeed * alphaLeft / betaLeft + frontRightWheelSpeed * alphaRight / betaRight);

  oneAxleSteeringMeasure.covariance = J * covariance * J.transpose();
}

//-----------------------------------------------------------------------------
void inverseKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OdometryFrame2FWS2RWD & odometryFrame,
  OneAxleSteeringMeasure & oneAxleSteeringMeasure)
{
  const double & wheelSpeedVariance = parameters.wheelLinearSpeedVariance;
  const double & wheelAngleVariance = parameters.wheelSteeringAngleVariance;

  const double wheelBase = parameters.frontWheelBase + parameters.rearWheelBase;
  const double frontHalfTrack = parameters.frontWheelTrack / 2;
  const double rearHalfTrack = parameters.rearWheelTrack / 2 + parameters.rearHubCarrierOffset;


  const double & frontLeftWheelAngle = odometryFrame.frontLeftWheelSteeringAngle;
  const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelLinearSpeed;
  double sinLeft = std::sin(frontLeftWheelAngle);
  double cosLeft = std::cos(frontLeftWheelAngle);
  double alphaLeft = cosLeft + sinLeft * frontHalfTrack / wheelBase;
  double betaLeft = cosLeft + sinLeft * (frontHalfTrack - rearHalfTrack) / wheelBase;
  double deltaLeft = 1 + rearHalfTrack * sinLeft / betaLeft;

  const double & frontRightWheelAngle = odometryFrame.frontRightWheelSteeringAngle;
  const double & rearRightWheelSpeed = odometryFrame.rearRightWheelLinearSpeed;
  double sinRight = std::sin(frontRightWheelAngle);
  double cosRight = std::cos(frontRightWheelAngle);
  double alphaRight = cosRight - sinRight * frontHalfTrack / wheelBase;
  double betaRight = cosRight - sinRight * (frontHalfTrack - rearHalfTrack) / wheelBase;
  double deltaRight = 1 - rearHalfTrack * sinRight / betaRight;

  double gamma = 1 / (1 + std::pow(0.5 * (sinLeft / alphaLeft + sinRight / alphaRight), 2));

  Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
  covariance(0, 0) = wheelSpeedVariance;
  covariance(1, 1) = wheelSpeedVariance;
  covariance(2, 2) = wheelAngleVariance;
  covariance(3, 3) = wheelAngleVariance;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2, 4);
  J(0, 0) = 0.5 * deltaLeft;
  J(0, 1) = 0.5 * deltaRight;
  J(0, 2) = rearLeftWheelSpeed * rearHalfTrack / (deltaLeft * deltaLeft);
  J(0, 3) = rearRightWheelSpeed * rearHalfTrack / (deltaRight * deltaRight);
  J(1, 2) = gamma / (alphaLeft * alphaLeft);
  J(1, 3) = gamma / (alphaLeft * alphaLeft);

  oneAxleSteeringMeasure.steeringAngle =
    std::atan(0.5 * (sinLeft / alphaLeft + sinRight / alphaRight));

  oneAxleSteeringMeasure.longitudinalSpeed =
    0.5 * (rearLeftWheelSpeed * deltaLeft + rearRightWheelSpeed * deltaRight);

  oneAxleSteeringMeasure.covariance = J * covariance * J.transpose();
}

//-----------------------------------------------------------------------------
void inverseKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OdometryFrame2FWS4WD & odometryFrame,
  OneAxleSteeringMeasure & oneAxleSteeringMeasure)
{
  const double & wheelSpeedVariance = parameters.wheelLinearSpeedVariance;
  const double & wheelAngleVariance = parameters.wheelSteeringAngleVariance;

  const double wheelBase = parameters.frontWheelBase + parameters.rearWheelBase;
  const double fronthubCarrierOffset = parameters.frontHubCarrierOffset;
  const double rearhubCarrierOffset = parameters.rearHubCarrierOffset;
  const double frontHalfTrack = parameters.frontWheelTrack / 2;
  const double rearHalfTrack = parameters.rearWheelTrack / 2 + rearhubCarrierOffset;

  const double & frontLeftWheelAngle = odometryFrame.frontLeftWheelSteeringAngle;
  const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelLinearSpeed;
  const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelLinearSpeed;
  double sinLeft = std::sin(frontLeftWheelAngle);
  double cosLeft = std::cos(frontLeftWheelAngle);
  double alphaLeft = cosLeft + sinLeft * frontHalfTrack / wheelBase;
  double betaLeft = cosLeft + sinLeft * (frontHalfTrack - rearHalfTrack) / wheelBase;
  double gammaLeft = 1 - fronthubCarrierOffset * sinLeft / wheelBase;
  double deltaLeft = 1 + rearHalfTrack * sinLeft / betaLeft;

  const double & frontRightWheelAngle = odometryFrame.frontRightWheelSteeringAngle;
  const double & frontRightWheelSpeed = odometryFrame.frontRightWheelLinearSpeed;
  const double & rearRightWheelSpeed = odometryFrame.rearRightWheelLinearSpeed;
  double sinRight = std::sin(frontRightWheelAngle);
  double cosRight = std::cos(frontRightWheelAngle);
  double alphaRight = cosRight - sinRight * frontHalfTrack / wheelBase;
  double betaRight = cosRight - sinRight * (frontHalfTrack - rearHalfTrack) / wheelBase;
  double gammaRight = 1 + fronthubCarrierOffset * sinRight / wheelBase;
  double deltaRight = 1 - rearHalfTrack * sinRight / betaRight;

  double epsilon = 1 / (1 + std::pow(0.5 * (sinLeft / alphaLeft + sinRight / alphaRight), 2));

  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(6, 6);
  covariance(0, 0) = wheelSpeedVariance;
  covariance(1, 1) = wheelSpeedVariance;
  covariance(2, 2) = wheelSpeedVariance;
  covariance(3, 3) = wheelSpeedVariance;
  covariance(4, 4) = wheelAngleVariance;
  covariance(5, 5) = wheelAngleVariance;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2, 6);

  J(0, 0) = 0.5 * alphaLeft / betaLeft;
  J(0, 1) = 0.5 * alphaRight / betaRight;
  J(0, 2) = 0.5 * deltaLeft;
  J(0, 3) = 0.5 * deltaRight;
  J(0, 4) += frontLeftWheelSpeed *
    ((-sinLeft + cosLeft * frontHalfTrack / wheelBase) - alphaLeft * fronthubCarrierOffset *
    cosLeft / wheelBase) / (gammaLeft * gammaLeft);
  J(0, 4) += rearLeftWheelSpeed * rearHalfTrack / (deltaLeft * deltaLeft);
  J(0, 5) += frontRightWheelSpeed *
    ((-sinRight - cosRight * frontHalfTrack / wheelBase) - alphaRight * fronthubCarrierOffset *
    cosRight / wheelBase) / (gammaRight * gammaRight);
  J(0, 5) += rearRightWheelSpeed * rearHalfTrack / (deltaRight * deltaRight);
  J(1, 4) = epsilon / (alphaLeft * alphaLeft);
  J(1, 5) = epsilon / (alphaLeft * alphaLeft);

  oneAxleSteeringMeasure.steeringAngle =
    std::atan(0.5 * (sinLeft / alphaLeft + sinRight / alphaRight));

  oneAxleSteeringMeasure.longitudinalSpeed =
    0.25 * (rearLeftWheelSpeed * deltaLeft + rearRightWheelSpeed * deltaRight) +
    0.25 * (frontLeftWheelSpeed * alphaLeft / gammaLeft +
    frontRightWheelSpeed * alphaRight / gammaRight);

  oneAxleSteeringMeasure.covariance = J * covariance * J.transpose();
}

}  // namespace core
}  // namespace romea
