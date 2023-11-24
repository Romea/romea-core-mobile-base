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


#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGCOMMANDLIMITS_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGCOMMANDLIMITS_HPP_

#include "romea_core_mobile_base/kinematic/CommandLimits.hpp"

namespace romea
{
namespace core
{

struct OneAxleSteeringCommandLimits
{
  OneAxleSteeringCommandLimits();

  OneAxleSteeringCommandLimits(
    const double & minimalLongitudinalSpeed,
    const double & maximalLongidudinalSpeed,
    const double & maximalSteeringAngle);

  Interval1D<double> longitudinalSpeed;
  Interval1D<double> steeringAngle;
};

std::ostream & operator<<(std::ostream & os, const OneAxleSteeringCommandLimits & limits);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGCOMMANDLIMITS_HPP_
