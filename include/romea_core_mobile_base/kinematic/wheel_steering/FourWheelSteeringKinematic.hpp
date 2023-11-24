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


#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__FOURWHEELSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__FOURWHEELSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"

namespace romea
{
namespace core
{

struct FourWheelSteeringKinematic
{
  struct Parameters
  {
    Parameters();
    double frontWheelBase;
    double rearWheelBase;
    double wheelTrack;
    double hubCarrierOffset;
    double maximalWheelLinearSpeed;
    double maximalWheelLinearAcceleration;
    double maximalWheelSteeringAngle;
    double maximalWheelSteeringAngularSpeed;
    double wheelLinearSpeedVariance;
    double wheelSteeringAngleVariance;
  };

  static double comptuteBeta(
    const double & linearSpeedXBodyAxis,
    const double & linearSpeedYBodyAxis);

  static double comptuteOrthogonalInstantaneousCurvature(
    const double & instantaneousCurvature,
    const double & beta);

  static double computeFrontSteeringAngle(
    const double & instantaneousCurvature,
    const double & frontWheelBase,
    const double & beta);

  static double computeRearSteeringAngle(
    const double & instantaneousCurvature,
    const double & rearWheelBase,
    const double & beta);
};

TwoAxleSteeringCommand clamp(
  const FourWheelSteeringKinematic::Parameters & parameters,
  const TwoAxleSteeringCommandLimits & userLimits,
  const TwoAxleSteeringCommand & command);

TwoAxleSteeringCommand clamp(
  const FourWheelSteeringKinematic::Parameters & parameters,
  const TwoAxleSteeringCommand & previousCommand,
  const TwoAxleSteeringCommand & curentCommand,
  const double & dt);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__FOURWHEELSTEERINGKINEMATIC_HPP_
