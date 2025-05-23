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


#ifndef ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWS2FWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWS2FWD_HPP_

#include "romea_core_mobile_base/simulation/SimulationControl2FWSxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2FWS2FWD.hpp"

namespace romea
{
namespace core
{

using  SimulationCommand2FWS2FWD = SimulationCommand2FWSxxx;
using  SimulationState2FWS2FWD = SimulationState2FWSxxx;

SimulationCommand2FWS2FWD  toSimulationCommand2FWS2FWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & rearTrack,
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const double & frontHubCarrierOffset,
  const double & rearHubCarrierOffset,
  const HardwareCommand2FWS2FWD & hardwareCommand);

SimulationState2FWS2FWD  toSimulationState2FWS2FWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & rearTrack,
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const double & frontHubCarrierOffset,
  const double & rearHubCarrierOffset,
  const HardwareState2FWS2FWD & hardwareState);

HardwareState2FWS2FWD toHardwareState2FWS2FWD(const SimulationState2FWS2FWD & simulationState);

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWS2FWD_HPP_
