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
#include "romea_core_mobile_base/info/MobileBaseInfo4WD.hpp"
#include <romea_core_common/math/Algorithm.hpp>

// std
#include <cmath>
#include <sstream>

namespace romea
{
namespace core
{

// //-----------------------------------------------------------------------------
// std::ostream & operator<<(std::ostream & os, const MobileBaseInfo4WD & baseInformation)
// {
//   os << "Base information:" << std::endl;
//   os << " geometry:" << std::endl;
//   os << baseInformation.geometry << std::endl;
//   os << " wheels speed control:" << std::endl;
//   os << baseInformation.wheelsSpeedControl << std::endl;
//   os << " inertia:" << std::endl;
//   os << baseInformation.inertia << std::endl;
//   os << " control point:" << std::endl;
//   os << "  " << baseInformation.controlPoint << std::endl;
//   return os;
// }

//-----------------------------------------------------------------------------
void to_kinematic_parameters(
  const MobileBaseInfo4WD & baseInformation,
  SkidSteeringKinematic::Parameters & kinematicParameters)
{
  const auto & geometry = baseInformation.geometry;
  const auto & wheelsCommand = baseInformation.wheelsSpeedControl.command;
  const auto & wheelsSensor = baseInformation.wheelsSpeedControl.sensor;

  if (!near(geometry.frontAxle.wheelsDistance, geometry.rearAxle.wheelsDistance)) {
    std::stringstream ss;
    ss << "Unable to convert base information to skid steering kinematic";
    ss << "because distance between wheels of front and rear axles are not equals";
    throw std::runtime_error(ss.str());
  }

  kinematicParameters.wheelTrack = geometry.frontAxle.wheelsDistance;
  kinematicParameters.maximalWheelLinearSpeed = wheelsCommand.maximalSpeed;
  kinematicParameters.wheelLinearSpeedVariance = std::pow(wheelsSensor.speedStd, 2);
  kinematicParameters.maximalWheelLinearAcceleration = wheelsCommand.maximalAcceleration;
}

void to_kinematic_parameters(
  const MobileBaseInfo4WD & baseInformation,
  MecanumWheelSteeringKinematic::Parameters & kinematicParameters)
{
  const auto & geometry = baseInformation.geometry;
  const auto & wheelsCommand = baseInformation.wheelsSpeedControl.command;
  const auto & wheelsSensor = baseInformation.wheelsSpeedControl.sensor;

  if (!near(geometry.frontAxle.wheelsDistance, geometry.rearAxle.wheelsDistance)) {
    std::stringstream ss;
    ss << "Unable to convert base information to omni steering kinematic";
    ss << "because distance between wheels of front and rear axles are not equals";
    throw std::runtime_error(ss.str());
  }

  kinematicParameters.wheelbase = geometry.axlesDistance;
  kinematicParameters.wheelTrack = geometry.frontAxle.wheelsDistance;
  kinematicParameters.maximalWheelLinearSpeed = wheelsCommand.maximalSpeed;
  kinematicParameters.wheelLinearSpeedVariance = std::pow(wheelsSensor.speedStd, 2);
  kinematicParameters.maximalWheelLinearAcceleration = wheelsCommand.maximalAcceleration;
}

}  // namespace core
}  // namespace romea
