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


#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__TWOWHEELSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__TWOWHEELSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp"

namespace romea
{
namespace core
{

class TwoWheelSteeringKinematic : public OneAxleSteeringKinematic
{
public:
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
    double maximalWheelSteeringAngle;
    double maximalWheelSteeringAngularSpeed;
    double wheelLinearSpeedVariance;
    double wheelSteeringAngleVariance;
  };


  static double computeInstantaneousCurvature(
    const double & leftInstantaneousCurvature,
    const double & rightInstantneousCurvature,
    const double & track);

  static double computeInstantaneousCurvature(
    const double & leftWheelSteeringAngle,
    const double & rightWheelSteeringAngle,
    const double & wheelbase,
    const double & track);

  static double computeSteeringAngle(
    const double & leftWheelSteeringAngle,
    const double & rightWheelSteeringAngle,
    const double & wheelbase,
    const double & track);


  static double computeMaximalInstantaneousCurvature(
    const double wheelbase,
    const double halfTrack,
    const double & maximalWheelSteeringAngle);


  static double computeLeftWheelSteeringAngle(
    const double & tanSteeringAngle,
    const double & instantaneousCurvature,
    const double & halfTrack);

  static double computeRightWheelSteeringAngle(
    const double & tanSteeringAngle,
    const double & instantaneousCurvature,
    const double & halfTrack);
};

OneAxleSteeringCommand clamp(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommandLimits & userLimits,
  const OneAxleSteeringCommand & command);

OneAxleSteeringCommand clamp(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & previousCommand,
  const OneAxleSteeringCommand & currentCommand,
  const double & dt);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__TWOWHEELSTEERINGKINEMATIC_HPP_
