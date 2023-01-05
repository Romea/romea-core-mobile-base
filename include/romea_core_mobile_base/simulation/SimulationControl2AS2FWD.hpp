// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2AS2FWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2AS2FWD_HPP_

#include "romea_core_mobile_base/simulation/SimulationControl2ASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2AS2FWD.hpp"

namespace romea
{

using SimulationCommand2AS2FWD = SimulationCommand2ASxxx;
using SimulationState2AS2FWD = SimulationState2ASxxx;

void toSimulation(
  const double & wheelbase,
  const double & frontTrack,
  const double & frontWheelRadius,
  const double & frontHubCarrierOffset,
  const double & rearTrack,
  const double & rearWheelRadius,
  const double & rearHubCarrierOffset,
  const HardwareCommand2AS2FWD & hardwareCommand,
  SimulationCommand2AS2FWD & simulationCommand);

void fromSimulation(
  const double & wheelbase,
  const double & front_track,
  const double & rearTrack,
  const SimulationState2AS2FWD & simulationState,
  HardwareState2AS2FWD & hardwareState);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2AS2FWD_HPP_
