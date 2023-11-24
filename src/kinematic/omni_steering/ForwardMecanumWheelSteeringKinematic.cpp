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


// std
#include <cmath>

// romea
#include "romea_core_mobile_base/kinematic/omni_steering/ForwardMecanumWheelSteeringKinematic.hpp"


namespace romea
{
namespace core
{

//--------------------------------------------------------------------------
void forwardKinematic(
  const MecanumWheelSteeringKinematic::Parameters & parameters,
  const OmniSteeringCommand & commandFrame,
  OdometryFrame4WD & odometryFrame)
{
  const double & longitudinalSpeed = commandFrame.longitudinalSpeed;
  const double & lateralSpeed = commandFrame.longitudinalSpeed;
  const double & angularSpeed = commandFrame.angularSpeed;
  const double halfWheebase = parameters.wheelbase / 2.;
  const double halfTrack = parameters.wheelTrack / 2.;

  odometryFrame.frontLeftWheelLinearSpeed = MecanumWheelSteeringKinematic::
    computeFrontLeftWheelLinearSpeed(
    longitudinalSpeed,
    lateralSpeed,
    angularSpeed,
    halfWheebase,
    halfTrack);

  odometryFrame.frontRightWheelLinearSpeed = MecanumWheelSteeringKinematic::
    computeFrontRightWheelLinearSpeed(
    longitudinalSpeed,
    lateralSpeed,
    angularSpeed,
    halfWheebase,
    halfTrack);

  odometryFrame.rearLeftWheelLinearSpeed = MecanumWheelSteeringKinematic::
    computeRearLeftWheelLinearSpeed(
    longitudinalSpeed,
    lateralSpeed,
    angularSpeed,
    halfWheebase,
    halfTrack);

  odometryFrame.rearRightWheelLinearSpeed = MecanumWheelSteeringKinematic::
    computeRearRightWheelLinearSpeed(
    longitudinalSpeed,
    lateralSpeed,
    angularSpeed,
    halfWheebase,
    halfTrack);
}

}  // namespace core
}  // namespace romea
