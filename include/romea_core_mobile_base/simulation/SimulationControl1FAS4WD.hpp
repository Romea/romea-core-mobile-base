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


#ifndef ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL1FAS4WD_HPP_
#define ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL1FAS4WD_HPP_

#include "romea_core_mobile_base/simulation/SimulationControl1FASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl1FAS4WD.hpp"

namespace romea
{
namespace core
{

using SimulationCommand1FAS4WD = SimulationCommand1FASxxx;
using SimulationState1FAS4WD = SimulationState1FASxxx;

void toSimulation(
  const double & wheelbase,
  const double & frontTrack,
  const HardwareCommand1FAS4WD & hardwareCommand,
  SimulationCommand1FAS4WD & simulationCommand);

void fromSimulation(
  const double & wheelbase,
  const double & frontTrack,
  const SimulationState1FAS4WD & simulationState,
  HardwareState1FAS4WD & hardwareState);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL1FAS4WD_HPP_
