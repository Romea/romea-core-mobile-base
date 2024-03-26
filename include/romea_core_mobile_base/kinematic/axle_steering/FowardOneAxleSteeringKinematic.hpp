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


#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__FOWARDONEAXLESTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__FOWARDONEAXLESTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS2FWD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS2RWD.hpp"
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS4WD.hpp"

namespace romea
{
namespace core
{

void forwardKinematic(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & commandFrame,
  OdometryFrame1FAS2FWD & odometryCommandFrame);

void forwardKinematic(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & commandFrame,
  OdometryFrame1FAS2RWD & odometryCommandFrame);

void forwardKinematic(
  const OneAxleSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & commandFrame,
  OdometryFrame1FAS4WD & odometryCommandFrame);

}  // namespace core
}  // namespace romea

#endif   // ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__FOWARDONEAXLESTEERINGKINEMATIC_HPP_
