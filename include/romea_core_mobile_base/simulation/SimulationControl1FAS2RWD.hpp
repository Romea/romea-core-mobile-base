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


#ifndef ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL1FAS2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL1FAS2RWD_HPP_

#include "romea_core_mobile_base/simulation/SimulationControl1FASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl1FAS2RWD.hpp"

namespace romea
{
namespace core
{

using SimulationCommand1FAS2RWD = SimulationCommand1FASxxx;
using SimulationState1FAS2RWD = SimulationState1FASxxx;

SimulationCommand1FAS2RWD toSimulationCommand1FAS2RWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & frontHubCarrierOffset,
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const HardwareCommand1FAS2RWD & hardwareCommand);

HardwareState1FAS2RWD toHardwareState1FAS2RWD(
  const double & wheelbase,
  const double & frontTrack,
  const SimulationState1FAS2RWD & simulationState);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL1FAS2RWD_HPP_
