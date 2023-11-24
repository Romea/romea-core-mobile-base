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


#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__FORWARDSKIDSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__FORWARDSKIDSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2TD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame2WD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame4WD.hpp"

namespace romea
{
namespace core
{

void forwardKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommand & commandFrame,
  OdometryFrame2TD & odometryCommandFrame);


void forwardKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommand & commandFrame,
  OdometryFrame2WD & odometryCommandFrame);


void forwardKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommand & commandFrame,
  OdometryFrame4WD & odometryCommandFrame);


void forwardKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommand & commandFrame,
  const OdometryFrame2WD & startOdometryFrame,
  OdometryFrame2WD & odometryCommandFrame);


void forwardKinematic(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommand & commandFrame,
  const OdometryFrame2WD & startOdometryFrame,
  OdometryFrame4WD & odometryCommandFrame);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__FORWARDSKIDSTEERINGKINEMATIC_HPP_
