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
#include "romea_core_mobile_base/kinematic/skid_steering/InverseSkidSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

namespace
{

//--------------------------------------------------------------------------
void inverseKinematicImpl(
  const double & wheelTrack,
  const double & leftWheelSpeed,
  const double & rightWheelSpeed,
  const double & wheelSpeedVariance,
  romea::SkidSteeringMeasure & skidSteeringMeasure)
{
  skidSteeringMeasure.longitudinalSpeed = romea::SkidSteeringKinematic::
    computeLinearSpeed(leftWheelSpeed, rightWheelSpeed);

  skidSteeringMeasure.angularSpeed = romea::SkidSteeringKinematic::
    computeAngularSpeed(leftWheelSpeed, rightWheelSpeed, wheelTrack);

  skidSteeringMeasure.covariance << 0.5, 1 / wheelTrack, 1 / wheelTrack,
      1 / (wheelTrack * wheelTrack);
  skidSteeringMeasure.covariance *= wheelSpeedVariance;
}

}  // namespace

namespace romea
{

//-----------------------------------------------------------------------------
void inverseKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const OdometryFrame2TD & odometryFrame,
  SkidSteeringMeasure & skidSteeringMeasure)
{
  inverseKinematicImpl(
    parameters.wheelTrack,
    odometryFrame.leftTrackLinearSpeed,
    odometryFrame.rightTrackLinearSpeed,
    parameters.wheelLinearSpeedVariance,
    skidSteeringMeasure);
}

//-----------------------------------------------------------------------------
void inverseKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const OdometryFrame2WD & odometryFrame,
  SkidSteeringMeasure & skidSteeringMeasure)
{
  inverseKinematicImpl(
    parameters.wheelTrack,
    odometryFrame.leftWheelLinearSpeed,
    odometryFrame.rightWheelLinearSpeed,
    parameters.wheelLinearSpeedVariance,
    skidSteeringMeasure);
}


//-----------------------------------------------------------------------------
void inverseKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const OdometryFrame4WD & odometryFrame,
  SkidSteeringMeasure & skidSteeringMeasure)
{
  double leftWheelSpeed = SkidSteeringKinematic::
    minWheelLinearSpeed(
    odometryFrame.frontLeftWheelLinearSpeed,
    odometryFrame.rearLeftWheelLinearSpeed);

  double rightWheelSpeed = SkidSteeringKinematic::
    minWheelLinearSpeed(
    odometryFrame.frontRightWheelLinearSpeed,
    odometryFrame.rearRightWheelLinearSpeed);

  inverseKinematicImpl(
    parameters.wheelTrack,
    leftWheelSpeed,
    rightWheelSpeed,
    parameters.wheelLinearSpeedVariance,
    skidSteeringMeasure);
}

}  // namespace romea
