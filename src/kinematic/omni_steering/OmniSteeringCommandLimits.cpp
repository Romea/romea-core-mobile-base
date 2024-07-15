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
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommandLimits.hpp"


namespace romea
{
namespace core
{


//--------------------------------------------------------------------------
OmniSteeringCommandLimits::OmniSteeringCommandLimits()
: longitudinalSpeed(),
  lateralSpeed(),
  angularSpeed()
{
}

//--------------------------------------------------------------------------
OmniSteeringCommandLimits::OmniSteeringCommandLimits(
  const double & minimalLongitudinalSpeed,
  const double & maximalLongidudinalSpeed,
  const double & maximalLateralSpeed,
  const double & maximalAngularSpeed)
: longitudinalSpeed(makeLongitudinalSpeedCommandLimits(minimalLongitudinalSpeed,
    maximalLongidudinalSpeed)),
  lateralSpeed(makeSymmetricCommandLimits(maximalLateralSpeed)),
  angularSpeed(makeSymmetricCommandLimits(maximalAngularSpeed))
{
}

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const OmniSteeringCommandLimits & limits)
{
  os << "Command limits : " << std::endl;
  os << " longitudinal speed : [" <<
    limits.longitudinalSpeed.lower() << " " <<
    limits.longitudinalSpeed.upper() << "]" << std::endl;
  os << " lateral speed : [" <<
    limits.lateralSpeed.lower() << " " <<
    limits.lateralSpeed.upper() << "]" << std::endl;
  os << " angular speed : [" <<
    limits.angularSpeed.lower() << " " <<
    limits.angularSpeed.upper() << "]";
  return os;
}

}  // namespace core
}  // namespace romea
