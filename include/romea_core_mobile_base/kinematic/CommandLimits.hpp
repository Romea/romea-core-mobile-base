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


#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__COMMANDLIMITS_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__COMMANDLIMITS_HPP_

#include <romea_core_common/math/Interval.hpp>

namespace romea
{
namespace core
{

Interval1D<double> makeSymmetricCommandLimits(const double & maximalAbsoluteCommand);

Interval1D<double> makeSteeringAngleCommandLimits(const double & maximalAbsoluteSteeringAngle);

Interval1D<double> makeLongitudinalSpeedCommandLimits(
  const double & maximalBackwardSpeed,
  const double & maximalForwardSpeed);

double clamp(const double & value, const Interval1D<double> & limits);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__COMMANDLIMITS_HPP_
