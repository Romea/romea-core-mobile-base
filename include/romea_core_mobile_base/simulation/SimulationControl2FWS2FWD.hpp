// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWS2FWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWS2FWD_HPP_

#include "romea_core_mobile_base/simulation/SimulationControl2FWSxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2FWS2FWD.hpp"

namespace romea
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

HardwareState2FWS2FWD toHardwareState2FWS2FWD(const SimulationState2FWS2FWD & simulationState);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2FWS2FWD_HPP_
