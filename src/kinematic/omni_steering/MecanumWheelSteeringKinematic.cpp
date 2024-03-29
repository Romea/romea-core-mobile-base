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


// romea core
#include <romea_core_common/math/Algorithm.hpp>

// std
#include <cmath>
#include <limits>
#include <algorithm>

// romea
#include "romea_core_mobile_base/kinematic/omni_steering/MecanumWheelSteeringKinematic.hpp"


namespace romea
{
namespace core
{


//--------------------------------------------------------------------------
MecanumWheelSteeringKinematic::Parameters::Parameters()
: wheelTrack(0),
  wheelbase(0),
  maximalWheelLinearSpeed(std::numeric_limits<double>::max()),
  maximalWheelLinearAcceleration(std::numeric_limits<double>::max()),
  wheelLinearSpeedVariance(0)
{
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeFrontLeftWheelLinearSpeed(
  const double & longitidutinalSpeed,
  const double & lateralSpeed,
  const double & angularSpeed,
  const double & halfWheelbase,
  const double & halfTrack)
{
  return longitidutinalSpeed - lateralSpeed - angularSpeed * (halfWheelbase + halfTrack);
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeFrontRightWheelLinearSpeed(
  const double & longitidutinalSpeed,
  const double & lateralSpeed,
  const double & angularSpeed,
  const double & halfWheelbase,
  const double & halfTrack)
{
  return longitidutinalSpeed + lateralSpeed + angularSpeed * (halfWheelbase + halfTrack);
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeRearLeftWheelLinearSpeed(
  const double & longitidutinalSpeed,
  const double & lateralSpeed,
  const double & angularSpeed,
  const double & halfWheelbase,
  const double & halfTrack)
{
  return longitidutinalSpeed + lateralSpeed - angularSpeed * (halfWheelbase + halfTrack);
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeRearRightWheelLinearSpeed(
  const double & longitidutinalSpeed,
  const double & lateralSpeed,
  const double & angularSpeed,
  const double & halfWheelbase,
  const double & halfTrack)
{
  return longitidutinalSpeed - lateralSpeed + angularSpeed * (halfWheelbase + halfTrack);
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeLongitudinalSpeed(
  const double & frontLeftWheelSpeed,
  const double & frontRightWheelSpeed,
  const double & rearLeftWheelSpeed,
  const double & rearRightWheelSpeed)
{
  return 0.25 * (frontLeftWheelSpeed + frontRightWheelSpeed +
         rearLeftWheelSpeed + rearRightWheelSpeed);
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeLateralSpeed(
  const double & frontLeftWheelSpeed,
  const double & frontRightWheelSpeed,
  const double & rearLeftWheelSpeed,
  const double & rearRightWheelSpeed)
{
  return 0.25 * ((frontRightWheelSpeed + rearLeftWheelSpeed) -
         (frontLeftWheelSpeed + rearRightWheelSpeed));
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeAngularSpeed(
  const double & frontLeftWheelSpeed,
  const double & frontRightWheelSpeed,
  const double & rearLeftWheelSpeed,
  const double & rearRightWheelSpeed,
  const double & halfWheelbase,
  const double & halfTrack)
{
  return 0.25 * ((frontRightWheelSpeed + rearRightWheelSpeed) -
         (rearLeftWheelSpeed + frontLeftWheelSpeed)) / (halfWheelbase + halfTrack);
}

//--------------------------------------------------------------------------
OmniSteeringCommand clamp(
  const MecanumWheelSteeringKinematic::Parameters & parameters,
  const OmniSteeringCommandLimits & userLimits,
  const OmniSteeringCommand & command)
{
  double alpha = parameters.wheelTrack + parameters.wheelbase;

  // Clamp angular speed
  double maximalAbsoluteAngularSpeed = 2 * parameters.maximalWheelLinearSpeed / alpha;

  maximalAbsoluteAngularSpeed = std::min(
    maximalAbsoluteAngularSpeed,
    userLimits.angularSpeed.upper());

  double angularSpeed = romea::core::clamp(
    command.angularSpeed,
    -maximalAbsoluteAngularSpeed,
    maximalAbsoluteAngularSpeed);

  // Clamp lateral speed
  double maximalAbsoluteSpeed = parameters.maximalWheelLinearSpeed -
    std::abs(angularSpeed) * alpha / 2.0;

  double maximalAbsboluteLateralSpeed =
    std::min(maximalAbsoluteSpeed, userLimits.lateralSpeed.upper());

  double lateralSpeed = clamp(
    command.lateralSpeed,
    -maximalAbsboluteLateralSpeed,
    maximalAbsboluteLateralSpeed);

  // Clamp longitudinal speed
  double longitudinalSpeed = command.longitudinalSpeed;

  double maximalAbsoluteLongitudinalSpeed = maximalAbsoluteSpeed - std::abs(lateralSpeed);

  double minimalLongitudinalSpeed =
    std::max(-maximalAbsoluteLongitudinalSpeed, userLimits.longitudinalSpeed.lower());

  longitudinalSpeed = std::max(longitudinalSpeed, minimalLongitudinalSpeed);

  double maximalLongitudinalSpeed =
    std::min(maximalAbsoluteLongitudinalSpeed, userLimits.longitudinalSpeed.upper());

  longitudinalSpeed = std::min(longitudinalSpeed, maximalLongitudinalSpeed);

  OmniSteeringCommand clampedCommand;
  clampedCommand.longitudinalSpeed = longitudinalSpeed;
  clampedCommand.lateralSpeed = lateralSpeed;
  clampedCommand.angularSpeed = angularSpeed;
  return clampedCommand;
}


//--------------------------------------------------------------------------
OmniSteeringCommand clamp(
  const MecanumWheelSteeringKinematic::Parameters & parameters,
  const OmniSteeringCommand & previousCommand,
  const OmniSteeringCommand & currentCommand,
  const double & dt)
{
  double alpha = parameters.wheelTrack + parameters.wheelbase;

  // Clamp angular speed
  double maximalAbsoluteAngularSpeed = 2 * parameters.maximalWheelLinearAcceleration / alpha;

  double angularAcceleration = currentCommand.angularSpeed - previousCommand.angularSpeed;

  angularAcceleration = romea::core::clamp(
    angularAcceleration,
    -maximalAbsoluteAngularSpeed,
    maximalAbsoluteAngularSpeed);

  // Clamp lateral speed
  double maximalAbsoluteLateralAcceleration = parameters.maximalWheelLinearAcceleration - std::abs(
    angularAcceleration) * alpha / 2.0;

  double lateralAcceleration = currentCommand.lateralSpeed - previousCommand.lateralSpeed;

  lateralAcceleration = romea::core::clamp(
    lateralAcceleration,
    -maximalAbsoluteLateralAcceleration,
    maximalAbsoluteLateralAcceleration);

  // Clamp longitudinal speed

  double maximalAbsoluteLongitudinalAcceleration = maximalAbsoluteLateralAcceleration - std::abs(
    lateralAcceleration);

  double longitudinalAcceleration = currentCommand.longitudinalSpeed -
    previousCommand.longitudinalSpeed;

  longitudinalAcceleration = romea::core::clamp(
    longitudinalAcceleration,
    -maximalAbsoluteLongitudinalAcceleration,
    maximalAbsoluteLongitudinalAcceleration);


  // return command
  OmniSteeringCommand clampedCommand;
  clampedCommand.longitudinalSpeed = previousCommand.longitudinalSpeed + longitudinalAcceleration *
    dt;
  clampedCommand.lateralSpeed = previousCommand.lateralSpeed + lateralAcceleration * dt;
  clampedCommand.angularSpeed = previousCommand.angularSpeed + angularAcceleration * dt;
  return clampedCommand;
}

}  // namespace core
}  // namespace romea
