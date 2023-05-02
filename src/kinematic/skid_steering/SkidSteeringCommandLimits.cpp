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
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommandLimits.hpp"
#include <romea_core_common/math/Algorithm.hpp>

namespace romea
{

//--------------------------------------------------------------------------
SkidSteeringCommandLimits::SkidSteeringCommandLimits()
: longitudinalSpeed(),
  angularSpeed()
{
}

//--------------------------------------------------------------------------
SkidSteeringCommandLimits::SkidSteeringCommandLimits(
  const double & minimalLongitudinalSpeed,
  const double & maximalLongidudinalSpeed,
  const double & maximalAngularSpeed)
: longitudinalSpeed(makeLongitudinalSpeedCommandLimits(minimalLongitudinalSpeed,
    maximalLongidudinalSpeed)),
  angularSpeed(makeSymmetricCommandLimits(maximalAngularSpeed))
{
}

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & os, const SkidSteeringCommandLimits & limits)
{
  os << "Command limits : " << std::endl;
  os << " linear speed : [" <<
    limits.longitudinalSpeed.lower() << " " <<
    limits.longitudinalSpeed.upper() << "]" << std::endl;
  os << " angular speed : [" <<
    limits.angularSpeed.lower() << " " <<
    limits.angularSpeed.upper() << "]";
  return os;
}

}  // namespace romea
