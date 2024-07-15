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
#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"

#include <iostream>
namespace romea
{
namespace core
{

//--------------------------------------------------------------------------
void forwardKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommand & commandFrame,
  OdometryFrame2TD & odometryCommandFrame)
{
  OdometryFrame2WD odometryCommandFrame2WD;
  forwardKinematic(parameters, commandFrame, odometryCommandFrame2WD);
  odometryCommandFrame.leftTrackLinearSpeed = odometryCommandFrame2WD.leftWheelLinearSpeed;
  odometryCommandFrame.rightTrackLinearSpeed = odometryCommandFrame2WD.rightWheelLinearSpeed;
}

//--------------------------------------------------------------------------
void forwardKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommand & commandFrame,
  OdometryFrame2WD & odometryCommandFrame)
{
  const double track = parameters.wheelTrack;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & angularSpeed = commandFrame.angularSpeed;

  odometryCommandFrame.leftWheelLinearSpeed = SkidSteeringKinematic::
    computeLeftWheelLinearSpeed(linearSpeed, angularSpeed, track);
  odometryCommandFrame.rightWheelLinearSpeed = SkidSteeringKinematic::
    computeRightWheelLinearSpeed(linearSpeed, angularSpeed, track);
}

//--------------------------------------------------------------------------
void forwardKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommand & commandFrame,
  OdometryFrame4WD & odometryCommandFrame)
{
  const double track = parameters.wheelTrack;
  const double & linearSpeed = commandFrame.longitudinalSpeed;
  const double & angularSpeed = commandFrame.angularSpeed;

  double leftWheelSpeed = SkidSteeringKinematic::
    computeLeftWheelLinearSpeed(linearSpeed, angularSpeed, track);
  double rightWheelSpeed = SkidSteeringKinematic::
    computeRightWheelLinearSpeed(linearSpeed, angularSpeed, track);

  odometryCommandFrame.frontLeftWheelLinearSpeed = leftWheelSpeed;
  odometryCommandFrame.frontRightWheelLinearSpeed = rightWheelSpeed;
  odometryCommandFrame.rearLeftWheelLinearSpeed = leftWheelSpeed;
  odometryCommandFrame.rearRightWheelLinearSpeed = rightWheelSpeed;
}

}  // namespace core
}  // namespace romea
