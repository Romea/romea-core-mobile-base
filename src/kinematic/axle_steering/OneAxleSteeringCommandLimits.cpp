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


// std
#include <ostream>

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommandLimits.hpp"
#include "romea_core_mobile_base/kinematic/CommandLimits.hpp"


namespace romea
{
namespace core
{

//--------------------------------------------------------------------------
OneAxleSteeringCommandLimits::OneAxleSteeringCommandLimits()
: longitudinalSpeed(),
  steeringAngle(-M_PI_2, M_PI_2)
{
}

//--------------------------------------------------------------------------
OneAxleSteeringCommandLimits::OneAxleSteeringCommandLimits(
  const double & minimalLongitudinalSpeed,
  const double & maximalLongidudinalSpeed,
  const double & maximalSteeringAngle)
: longitudinalSpeed(makeLongitudinalSpeedCommandLimits(minimalLongitudinalSpeed,
    maximalLongidudinalSpeed)),
  steeringAngle(makeSteeringAngleCommandLimits(maximalSteeringAngle))
{
}

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const OneAxleSteeringCommandLimits & limits)
{
  os << "Command limits : " << std::endl;
  os << " longitudinal speed : [" <<
    limits.longitudinalSpeed.lower() << " " <<
    limits.longitudinalSpeed.upper() << "]" << std::endl;
  os << " stering angle : [" <<
    limits.steeringAngle.lower() << " " <<
    limits.steeringAngle.upper() << "]";
  return os;
}

}  // namespace core
}  // namespace romea
