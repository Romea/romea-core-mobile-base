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


// romea core
#include <romea_core_common/math/Algorithm.hpp>

// std
#include <cmath>

// romea
#include "romea_core_mobile_base/kinematic/CommandLimits.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
Interval1D<double> makeSymmetricCommandLimits(const double & maximalAbsoluteCommand)
{
  return Interval1D<double>(-maximalAbsoluteCommand, maximalAbsoluteCommand);
}

//-----------------------------------------------------------------------------
Interval1D<double> makeSteeringAngleCommandLimits(const double & maximalAbsoluteSteeringAngle)
{
  if (maximalAbsoluteSteeringAngle < 0 || maximalAbsoluteSteeringAngle > M_PI_2) {
    std::stringstream ss;
    ss << " Steering angle limits:";
    ss << " Unable to set upper angle to ";
    ss << maximalAbsoluteSteeringAngle;
    ss << " because this value is not in range [0 pi/2]";
    throw std::runtime_error(ss.str());
  }

  return Interval1D<double>(-maximalAbsoluteSteeringAngle, maximalAbsoluteSteeringAngle);
}


//-----------------------------------------------------------------------------
Interval1D<double> makeLongitudinalSpeedCommandLimits(
  const double & maximalBackwardSpeed,
  const double & maximalForwardSpeed)
{
  if (maximalBackwardSpeed > 0) {
    std::stringstream ss;
    ss << " Longitudinal speed limits : ";
    ss << " Unable to set lower speed to ";
    ss << maximalBackwardSpeed;
    ss << " because its a positive value";
    throw std::runtime_error(ss.str());
  }

  if (maximalForwardSpeed < 0) {
    std::stringstream ss;
    ss << " Longitudinal speed limits : ";
    ss << " Unable to set upper speed to ";
    ss << maximalForwardSpeed;
    ss << " because it's a negative value";
    throw std::runtime_error(ss.str());
  }

  return Interval1D<double>(maximalBackwardSpeed, maximalForwardSpeed);
}

//-----------------------------------------------------------------------------
double clamp(const double & value, const Interval1D<double> & limits)
{
  return clamp(value, limits.lower(), limits.upper());
}

}  // namespace romea
