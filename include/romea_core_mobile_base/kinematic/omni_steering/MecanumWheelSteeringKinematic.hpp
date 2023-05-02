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


#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__MECANUMWHEELSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__MECANUMWHEELSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommandLimits.hpp"

namespace romea
{

struct MecanumWheelSteeringKinematic
{
  struct Parameters
  {
    Parameters();
    double wheelTrack;
    double wheelbase;
    double maximalWheelLinearSpeed;
    double maximalWheelLinearAcceleration;
    double wheelLinearSpeedVariance;
  };


  static double computeFrontLeftWheelLinearSpeed(
    const double & longitidutinalSpeed,
    const double & lateralSpeed,
    const double & angularSpeed,
    const double & halfWheelbase,
    const double & halfTrack);

  static double computeFrontRightWheelLinearSpeed(
    const double & longitidutinalSpeed,
    const double & lateralSpeed,
    const double & angularSpeed,
    const double & halfWheelbase,
    const double & halfTrack);

  static double computeRearLeftWheelLinearSpeed(
    const double & longitidutinalSpeed,
    const double & lateralSpeed,
    const double & angularSpeed,
    const double & halfWheelbase,
    const double & halfTrack);

  static double computeRearRightWheelLinearSpeed(
    const double & longitidutinalSpeed,
    const double & lateralSpeed,
    const double & angularSpeed,
    const double & halfWheelbase,
    const double & halfTrack);


  static double computeLongitudinalSpeed(
    const double & frontLeftWheelSpeed,
    const double & frontRightWheelSpeed,
    const double & rearLeftWheelSpeed,
    const double & rearRightWheelSpeed);

  static double computeLateralSpeed(
    const double & frontLeftWheelSpeed,
    const double & frontRightWheelSpeed,
    const double & rearLeftWheelSpeed,
    const double & rearRightWheelSpeed);


  static double computeAngularSpeed(
    const double & frontLeftWheelSpeed,
    const double & frontRightWheelSpeed,
    const double & rearLeftWheelSpeed,
    const double & rearRightWheelSpeed,
    const double & halfWheelbase,
    const double & halfTrack);
};

OmniSteeringCommand clamp(
  const MecanumWheelSteeringKinematic::Parameters & parameters,
  const OmniSteeringCommandLimits & userLimits,
  const OmniSteeringCommand & command);

OmniSteeringCommand clamp(
  const MecanumWheelSteeringKinematic::Parameters & parameters,
  const OmniSteeringCommand & previousCommand,
  const OmniSteeringCommand & currentCommand,
  const double & dt);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__MECANUMWHEELSTEERINGKINEMATIC_HPP_
