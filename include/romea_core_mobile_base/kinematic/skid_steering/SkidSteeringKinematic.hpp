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


#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__SKIDSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__SKIDSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommandLimits.hpp"

namespace romea
{
namespace core
{

struct SkidSteeringKinematic
{
  struct Parameters
  {
    Parameters();
    double wheelTrack;
    double maximalWheelLinearSpeed;
    double maximalWheelLinearAcceleration;
    double wheelLinearSpeedVariance;
  };

  static double computeLinearSpeed(
    const double & leftWheelLinearSpeed,
    const double & rightWheelLinearSpeed);

  static double computeAngularSpeed(
    const double & leftWheelLinearSpeed,
    const double & rightWheelLinearSpeed,
    const double & track);

  static double computeInstantaneousCurvature(
    const double & leftWheelLinearSpeed,
    const double & rightWheelLinearSpeed,
    const double & track);

  static double computeLeftWheelLinearSpeed(
    const double & linearSpeed,
    const double & angularSpeed,
    const double & wheelTrack);

  static double computeRightWheelLinearSpeed(
    const double & linearSpeed,
    const double & angularSpeed,
    const double & wheelTrack);

  static double minWheelLinearSpeed(
    const double & frontWheelLinearSpeed,
    const double & rearWheelLinearSpeed);
};

SkidSteeringCommand clamp(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommandLimits & userLimits,
  const SkidSteeringCommand & command);

SkidSteeringCommand clamp(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommand & previousCommand,
  const SkidSteeringCommand & currentCommand,
  const double & dt);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__SKIDSTEERINGKINEMATIC_HPP_
