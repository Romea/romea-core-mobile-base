// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWS2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWS2RWD_HPP_

#include "romea_core_mobile_base/simulation/SimulationControl2FWSxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2FWS2RWD.hpp"

namespace romea
{

using  SimulationCommand2FWS2RWD = SimulationCommand2FWSxxx;
using  SimulationState2FWS2RWD = SimulationState2FWSxxx;

SimulationCommand2FWS2RWD toSimulationCommand2FWS2RWD(
  const double & wheelbase,
  const double & frontTrack,
  const double & frontHubCarrierOffset,
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const HardwareCommand2FWS2RWD & hardwareCommand);

HardwareState2FWS2RWD toHardwareState2FWS2RWD(const SimulationState2FWS2RWD & simulationState);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWS2RWD_HPP_
