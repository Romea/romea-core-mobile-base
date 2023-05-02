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
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void forwardKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & commandFrame,
  OdometryFrame2FWS2FWD & commandOdometryFrame)
{
  const double halfTrack = parameters.frontWheelTrack / 2;
  const double wheelBase = parameters.rearWheelBase + parameters.frontWheelBase;
  const double & hubCarrierOffset = parameters.frontHubCarrierOffset;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & steeringAngle = commandFrame.steeringAngle;

  double tanSteeringAngle = std::tan(steeringAngle);
  double instantaneousCurvature = tanSteeringAngle / wheelBase;

  double frontLeftWheelAngle = TwoWheelSteeringKinematic::
    computeLeftWheelSteeringAngle(
    tanSteeringAngle,
    instantaneousCurvature,
    halfTrack);

  double frontRightWheelAngle = TwoWheelSteeringKinematic::
    computeRightWheelSteeringAngle(
    tanSteeringAngle,
    instantaneousCurvature,
    halfTrack);

  double frontLeftWheelSpeed = OneAxleSteeringKinematic::
    computeLeftWheelLinearSpeed(
    linearSpeed,
    tanSteeringAngle,
    instantaneousCurvature,
    hubCarrierOffset,
    halfTrack);

  double frontRightWheelSpeed = OneAxleSteeringKinematic::
    computeRightWheelLinearSpeed(
    linearSpeed,
    tanSteeringAngle,
    instantaneousCurvature,
    hubCarrierOffset,
    halfTrack);

  commandOdometryFrame.frontLeftWheelLinearSpeed = frontLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelSteeringAngle = frontLeftWheelAngle;
  commandOdometryFrame.frontRightWheelLinearSpeed = frontRightWheelSpeed;
  commandOdometryFrame.frontRightWheelSteeringAngle = frontRightWheelAngle;
}

//-----------------------------------------------------------------------------
void forwardKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & commandFrame,
  OdometryFrame2FWS2RWD & commandOdometryFrame)
{
  const double wheelBase = parameters.rearWheelBase + parameters.frontWheelBase;
  const double frontTrack = parameters.frontWheelTrack;
  const double rearTrack = parameters.rearWheelTrack;
  const double & rearHubCarrierOffset = parameters.rearHubCarrierOffset;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & steeringAngle = commandFrame.steeringAngle;

  double tanSteeringAngle = std::tan(steeringAngle);
  double angularSpeed = tanSteeringAngle / wheelBase * linearSpeed;
  double instantaneousCurvature = tanSteeringAngle / wheelBase;


  double frontLeftWheelAngle = TwoWheelSteeringKinematic::
    computeLeftWheelSteeringAngle(
    tanSteeringAngle,
    instantaneousCurvature,
    frontTrack / 2);

  double frontRightWheelAngle = TwoWheelSteeringKinematic::
    computeRightWheelSteeringAngle(
    tanSteeringAngle,
    instantaneousCurvature,
    frontTrack / 2);

  double rearLeftWheelSpeed = SkidSteeringKinematic::
    computeLeftWheelLinearSpeed(
    linearSpeed,
    angularSpeed,
    rearTrack + 2 * rearHubCarrierOffset);

  double rearRightWheelSpeed = SkidSteeringKinematic::
    computeRightWheelLinearSpeed(
    linearSpeed,
    angularSpeed,
    rearTrack + 2 * rearHubCarrierOffset);

  commandOdometryFrame.rearLeftWheelLinearSpeed = rearLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelSteeringAngle = frontLeftWheelAngle;
  commandOdometryFrame.rearRightWheelLinearSpeed = rearRightWheelSpeed;
  commandOdometryFrame.frontRightWheelSteeringAngle = frontRightWheelAngle;
}

//-----------------------------------------------------------------------------
void forwardKinematic(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & commandFrame,
  OdometryFrame2FWS4WD & commandOdometryFrame)
{
  const double wheelBase = parameters.rearWheelBase + parameters.frontWheelBase;
  const double frontTrack = parameters.frontWheelTrack;
  const double rearTrack = parameters.rearWheelTrack;
  const double & rearHubCarrierOffset = parameters.rearHubCarrierOffset;
  const double & frontHubCarrierOffset = parameters.frontHubCarrierOffset;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & steeringAngle = commandFrame.steeringAngle;

  double tanSteeringAngle = std::tan(steeringAngle);
  double angularSpeed = tanSteeringAngle / wheelBase * linearSpeed;
  double instantaneousCurvature = tanSteeringAngle / wheelBase;

  double frontLeftWheelAngle = TwoWheelSteeringKinematic::
    computeLeftWheelSteeringAngle(
    tanSteeringAngle,
    instantaneousCurvature,
    frontTrack / 2);

  double frontRightWheelAngle = TwoWheelSteeringKinematic::
    computeRightWheelSteeringAngle(
    tanSteeringAngle,
    instantaneousCurvature,
    frontTrack / 2);

  double frontLeftWheelSpeed = OneAxleSteeringKinematic::
    computeLeftWheelLinearSpeed(
    linearSpeed,
    tanSteeringAngle,
    instantaneousCurvature,
    frontHubCarrierOffset,
    frontTrack / 2);

  double frontRightWheelSpeed = OneAxleSteeringKinematic::
    computeRightWheelLinearSpeed(
    linearSpeed,
    tanSteeringAngle,
    instantaneousCurvature,
    frontHubCarrierOffset,
    frontTrack / 2);

  double rearLeftWheelSpeed = SkidSteeringKinematic::
    computeLeftWheelLinearSpeed(
    linearSpeed,
    angularSpeed,
    rearTrack + 2 * rearHubCarrierOffset);

  double rearRightWheelSpeed = SkidSteeringKinematic::
    computeRightWheelLinearSpeed(
    linearSpeed,
    angularSpeed,
    rearTrack + 2 * rearHubCarrierOffset);

  commandOdometryFrame.rearLeftWheelLinearSpeed = rearLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelLinearSpeed = frontLeftWheelSpeed;
  commandOdometryFrame.frontLeftWheelSteeringAngle = frontLeftWheelAngle;
  commandOdometryFrame.rearRightWheelLinearSpeed = rearRightWheelSpeed;
  commandOdometryFrame.frontRightWheelLinearSpeed = frontRightWheelSpeed;
  commandOdometryFrame.frontRightWheelSteeringAngle = frontRightWheelAngle;
}

}  // namespace romea
