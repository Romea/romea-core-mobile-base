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
#include "romea_core_mobile_base/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
void forwardKinematic(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & commandFrame,
  OdometryFrame1FAS2FWD & odometryCommandFrame)
{
  const double halfWheelTrack = parameters.frontWheelTrack / 2.;
  const double wheelBase = parameters.frontWheelBase + parameters.rearWheelBase;
  const double hubCarrierOffset = parameters.frontHubCarrierOffset;

  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & steeringAngle = commandFrame.steeringAngle;
  double tanSteeringAngle = std::tan(steeringAngle);
  double instantaneousCurvature = tanSteeringAngle / wheelBase;

  double frontLeftWheelSpeed = OneAxleSteeringKinematic::
    computeLeftWheelLinearSpeed(
    linearSpeed,
    tanSteeringAngle,
    instantaneousCurvature,
    hubCarrierOffset,
    halfWheelTrack);

  double frontRightWheelSpeed = OneAxleSteeringKinematic::
    computeRightWheelLinearSpeed(
    linearSpeed,
    tanSteeringAngle,
    instantaneousCurvature,
    hubCarrierOffset,
    halfWheelTrack);

  odometryCommandFrame.frontAxleSteeringAngle = steeringAngle;
  odometryCommandFrame.frontLeftWheelLinearSpeed = frontLeftWheelSpeed;
  odometryCommandFrame.frontRightWheelLinearSpeed = frontRightWheelSpeed;
}

//-----------------------------------------------------------------------------
void forwardKinematic(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & commandFrame,
  OdometryFrame1FAS2RWD & odometryCommandFrame)
{
  const double & wheelTrack = parameters.rearWheelTrack + 2 * parameters.rearHubCarrierOffset;
  const double & wheelBase = parameters.frontWheelBase + parameters.rearWheelBase;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & steeringAngle = commandFrame.steeringAngle;

  double instantaneousCurvature = std::tan(steeringAngle) / wheelBase;
  double rearLeftWheelSpeed = SkidSteeringKinematic::computeLeftWheelLinearSpeed(
    linearSpeed, instantaneousCurvature * linearSpeed, wheelTrack);
  double rearRightWheelSpeed = SkidSteeringKinematic::computeRightWheelLinearSpeed(
    linearSpeed, instantaneousCurvature * linearSpeed, wheelTrack);

  odometryCommandFrame.frontAxleSteeringAngle = steeringAngle;
  odometryCommandFrame.rearLeftWheelLinearSpeed = rearLeftWheelSpeed;
  odometryCommandFrame.rearRightWheelLinearSpeed = rearRightWheelSpeed;
}

}  // namespace core
}  // namespace romea
