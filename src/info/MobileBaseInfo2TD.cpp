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
#include "romea_core_mobile_base/info/MobileBaseInfo2TD.hpp"

// std
#include <cmath>

namespace romea
{

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2TD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " type:"<< std::endl;
//  os << "  2TD" << std::endl;
//  os << " geometry:" << std::endl;
//  os << baseInformation.geometry << std::endl;
//  os << " tracks speed control:" << std::endl;
//  os << baseInformation.tracksSpeedControl<< std::endl;
//  os << " inertia:" << std::endl;
//  os << baseInformation.inertia<< std::endl;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(
  const MobileBaseInfo2TD & base_information,
  SkidSteeringKinematic::Parameters & kinematic_parameters)
{
  const auto & geometry = base_information.geometry;
  const auto & tracksCommand = base_information.tracksSpeedControl.command;
  const auto & tracksSensor = base_information.tracksSpeedControl.sensor;

  kinematic_parameters.wheelTrack = geometry.tracksDistance;
  kinematic_parameters.maximalWheelLinearSpeed = tracksCommand.maximalSpeed;
  kinematic_parameters.wheelLinearSpeedVariance = std::pow(tracksSensor.speedStd, 2);
  kinematic_parameters.maximalWheelLinearAcceleration = tracksCommand.maximalAcceleration;
}

}  // namespace romea
