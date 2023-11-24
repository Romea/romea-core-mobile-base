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


#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGKINEMATIC_HPP_

// std
#include <cmath>

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommandLimits.hpp"


namespace romea
{
namespace core
{

struct OneAxleSteeringKinematic
{
  struct Parameters
  {
    Parameters();
    double frontWheelBase;
    double rearWheelBase;
    double frontWheelTrack;
    double rearWheelTrack;
    double frontHubCarrierOffset;
    double rearHubCarrierOffset;
    double frontMaximalWheelLinearSpeed;
    double rearMaximalWheelLinearSpeed;
    double maximalWheelLinearAcceleration;
    double maximalSteeringAngle;
    double maximalSteeringAngularSpeed;
    double wheelLinearSpeedVariance;
    double steeringAngleVariance;
  };


  static double computeBeta(
    const double & tanSteeringAngle,
    const double & frontWheelBase,
    const double & rearWheelBase);

  static double computeInstantaneousCurvature(
    const double & tanSteeringAngle,
    const double & wheelBase);


  static double computeSteeringAngle(
    const double & instantaneousCurvature,
    const double & wheelBase);


  static double computeAngularSpeed(
    const double & linearSpeed,
    const double & instantaneousCurvature);


  static double computeWheelLinearSpeedRatio(
    const double & tanSteeringAngle,
    const double & instaneousCurvature,
    const double & hubCarrierOffset,
    const double & halfTrack);

  static double computeLeftWheelLinearSpeed(
    const double & linearSpeed,
    const double & tanSteeringAngle,
    const double & instaneousCurvature,
    const double & hubCarrierOffset,
    const double & halfTrack);

  static double computeRightWheelLinearSpeed(
    const double & linearSpeed,
    const double & tanSteeringAngle,
    const double & instaneousCurvature,
    const double & hubCarrierOffset,
    const double & halfTrack);

  static double computeLinearSpeed(
    const double & leftWheelSpeed,
    const double & rightWheelSpeed,
    const double & tanSteeringAngle,
    const double & instaneousCurvature,
    const double & hubCarrierOffset,
    const double & halfTrack);

  static OneAxleSteeringCommand clamp(
    const double & wheelbase,
    const double & frontHalfTrack,
    const double & rearHalfTrack,
    const double & frontHubCarrierOffset,
    const double & rearHubCarrierOffset,
    const double & maximalSteeringAngle,
    const double & frontMaximalWheelSpeed,
    const double & rearMaximalWheelSpeed,
    const OneAxleSteeringCommandLimits & userLimits,
    const OneAxleSteeringCommand & command);

  static OneAxleSteeringCommand clamp(
    const double & wheelbase,
    const double & frontHalfTrack,
    const double & rearHalfTrack,
    const double & frontHubCarrierOffset,
    const double & rearHubCarrierOffset,
    const double & maximalSteeringAngularSpeed,
    const double & maximalWheelAcceleration,
    const OneAxleSteeringCommand & previousCommand,
    const OneAxleSteeringCommand & currentCommand,
    const double & dt);
};


OneAxleSteeringCommand clamp(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommandLimits & userLimits,
  const OneAxleSteeringCommand & command);

OneAxleSteeringCommand clamp(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & previousCommand,
  const OneAxleSteeringCommand & currentCommand,
  const double & dt);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGKINEMATIC_HPP_
