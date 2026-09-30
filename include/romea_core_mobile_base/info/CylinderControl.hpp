// Copyright 2026 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

#ifndef ROMEA_CORE_MOBILE_BASE__INFO__CYLINDERCONTROL_HPP_
#define ROMEA_CORE_MOBILE_BASE__INFO__CYLINDERCONTROL_HPP_

#include <limits>

namespace romea
{
namespace core
{

struct CylinderPositionSensor
{
  double position_std{0};
};

struct CylinderPositionCommandLimits
{
  double minimal_length;
  double maximal_length;
  double maximal_speed;
  double maximal_linear_acceleration{std::numeric_limits<double>::max()};
};

struct CylinderPositionControl
{
  CylinderPositionSensor sensor;
  CylinderPositionCommandLimits command;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__INFO__CYLINDERCONTROL_HPP_
