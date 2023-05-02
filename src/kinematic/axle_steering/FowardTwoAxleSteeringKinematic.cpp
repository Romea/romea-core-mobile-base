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
#include <romea_core_common/math/Algorithm.hpp>
#include "romea_core_mobile_base/kinematic/axle_steering/FowardTwoAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
void forwardKinematic(
  const TwoAxleSteeringKinematic::Parameters & parameters,
  const TwoAxleSteeringCommand & commandFrame,
  OdometryFrame2AS4WD & odometryCommandFrame)
{
  double wheelbase = parameters.frontWheelBase + parameters.rearWheelBase;

  const double & linearSpeed = commandFrame.longitudinalSpeed;
  double frontSteeringAngle = commandFrame.frontSteeringAngle;
  double tanFrontSteeringAngle = std::tan(frontSteeringAngle);
  const double frontHubCarrierOffset = parameters.frontHubCarrierOffset;
  const double frontHalfWheelTrack = parameters.frontWheelTrack / 2;

  double rearSteeringAngle = commandFrame.rearSteeringAngle;
  double tanRearSteeringAngle = std::tan(rearSteeringAngle);
  const double rearHubCarrierOffset = parameters.rearHubCarrierOffset;
  const double rearHalfTrack = parameters.rearWheelTrack / 2;

  double instantaneousCurvature = (tanFrontSteeringAngle - tanRearSteeringAngle) / wheelbase;

  double frontLeftWheelSpeed = OneAxleSteeringKinematic::
    computeLeftWheelLinearSpeed(
    linearSpeed,
    tanFrontSteeringAngle,
    instantaneousCurvature,
    frontHubCarrierOffset,
    frontHalfWheelTrack);

  double frontRightWheelSpeed = OneAxleSteeringKinematic::
    computeRightWheelLinearSpeed(
    linearSpeed,
    tanFrontSteeringAngle,
    instantaneousCurvature,
    frontHubCarrierOffset,
    frontHalfWheelTrack);


  double rearLeftWheelSpeed = OneAxleSteeringKinematic::
    computeLeftWheelLinearSpeed(
    linearSpeed,
    tanRearSteeringAngle,
    instantaneousCurvature,
    rearHubCarrierOffset,
    rearHalfTrack);

  double rearRightWheelSpeed = OneAxleSteeringKinematic::
    computeRightWheelLinearSpeed(
    linearSpeed,
    tanRearSteeringAngle,
    instantaneousCurvature,
    rearHubCarrierOffset,
    rearHalfTrack);

  assert(sign(frontLeftWheelSpeed) == sign(linearSpeed));
  assert(sign(frontRightWheelSpeed) == sign(linearSpeed));
  assert(sign(rearLeftWheelSpeed) == sign(linearSpeed));
  assert(sign(rearRightWheelSpeed) == sign(linearSpeed));

  odometryCommandFrame.frontAxleSteeringAngle = frontSteeringAngle;
  odometryCommandFrame.frontLeftWheelLinearSpeed = frontLeftWheelSpeed;
  odometryCommandFrame.frontRightWheelLinearSpeed = frontRightWheelSpeed;
  odometryCommandFrame.rearAxleSteeringAngle = rearSteeringAngle;
  odometryCommandFrame.rearLeftWheelLinearSpeed = rearLeftWheelSpeed;
  odometryCommandFrame.rearRightWheelLinearSpeed = rearRightWheelSpeed;
}

}  // namespace romea
