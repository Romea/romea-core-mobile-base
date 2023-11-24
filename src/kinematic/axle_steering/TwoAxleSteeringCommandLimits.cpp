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
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommandLimits.hpp"


namespace romea
{
namespace core
{

//--------------------------------------------------------------------------
TwoAxleSteeringCommandLimits::TwoAxleSteeringCommandLimits()
: longitudinalSpeed(),
  frontSteeringAngle(-M_PI_2, M_PI_2),
  rearSteeringAngle(-M_PI_2, M_PI_2)
{
}

//--------------------------------------------------------------------------
TwoAxleSteeringCommandLimits::TwoAxleSteeringCommandLimits(
  const double & minimalLongitudinalSpeed,
  const double & maximalLongidudinalSpeed,
  const double & maximalFrontSteeringAngle,
  const double & maximalRearSteeringAngle)
: longitudinalSpeed(makeLongitudinalSpeedCommandLimits(minimalLongitudinalSpeed,
    maximalLongidudinalSpeed)),
  frontSteeringAngle(makeSteeringAngleCommandLimits(maximalFrontSteeringAngle)),
  rearSteeringAngle(makeSteeringAngleCommandLimits(maximalRearSteeringAngle))
{
}

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const TwoAxleSteeringCommandLimits & limits)
{
  os << "Command limits : " << std::endl;
  os << " longitudinal speed : [" <<
    limits.longitudinalSpeed.lower() << " " <<
    limits.longitudinalSpeed.upper() << "]" << std::endl;
  os << " front stering angle : [" <<
    limits.frontSteeringAngle.lower() << " " <<
    limits.frontSteeringAngle.upper() << "]";
  os << " rear stering angle : [" <<
    limits.rearSteeringAngle.lower() << " " <<
    limits.rearSteeringAngle.upper() << "]";
  return os;
}

}  // namespace core
}  // namespace romea
