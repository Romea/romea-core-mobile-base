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


#ifndef ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO4WS4WD_HPP_
#define ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO4WS4WD_HPP_

#include "romea_core_mobile_base/info/MobileBaseControl.hpp"
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"
#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"

namespace romea
{
namespace core
{

struct MobileBaseInfo4WS4WD
{
  TwoAxles<WheeledAxle, WheeledAxle> geometry;
  SteeringAngleControl wheelsSteeringControl;
  WheelSpeedControl wheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

// std::ostream & operator<<(std::ostream & os, const MobileBaseInfo4WS4WD & baseInformation);

void to_kinematic_parameters(
  const MobileBaseInfo4WS4WD & baseInformation,
  FourWheelSteeringKinematic::Parameters & kinematicParameters);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO4WS4WD_HPP_
