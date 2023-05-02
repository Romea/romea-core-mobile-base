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
#include "romea_core_mobile_base/info/MobileBaseInfo2WD.hpp"

// std
#include <cmath>

namespace romea
{

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2WD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " geometry:" << std::endl;
//  os << baseInformation.geometry << std::endl;
//  os << " tracks speed control:" << std::endl;
//  os << baseInformation.wheelsSpeedControl<< std::endl;
//  os << " inertia:" << std::endl;
//  os << baseInformation.inertia<< std::endl;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(
  const MobileBaseInfo2WD & baseInformation,
  SkidSteeringKinematic::Parameters & kinematicParameters)
{
  const auto & geometry = baseInformation.geometry;
  const auto & wheelsCommand = baseInformation.wheelsSpeedControl.command;
  const auto & wheelsSensor = baseInformation.wheelsSpeedControl.sensor;

  kinematicParameters.wheelTrack = geometry.wheelsDistance;
  kinematicParameters.maximalWheelLinearSpeed = wheelsCommand.maximalSpeed;
  kinematicParameters.wheelLinearSpeedVariance = std::pow(wheelsSensor.speedStd, 2);
  kinematicParameters.maximalWheelLinearAcceleration = wheelsCommand.maximalAcceleration;
}

}
