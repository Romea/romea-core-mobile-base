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


#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGCOMMAND_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGCOMMAND_HPP_

// romea
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommandLimits.hpp"

namespace romea
{
namespace core
{

struct OmniSteeringCommand
{
  OmniSteeringCommand();

  OmniSteeringCommand(
    const double & longitudinalSpeed,
    const double & lateralSpeed,
    const double & angularSpeed);

  double longitudinalSpeed;
  double lateralSpeed;
  double angularSpeed;
};

std::ostream & operator<<(std::ostream & os, const OmniSteeringCommand & command);

OmniSteeringCommand clamp(
  const OmniSteeringCommand & command,
  const OmniSteeringCommandLimits & limits);

bool isValid(const OmniSteeringCommand & commaand);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGCOMMAND_HPP_
