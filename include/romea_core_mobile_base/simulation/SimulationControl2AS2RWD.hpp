// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2AS2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2AS2RWD_HPP_

#include "romea_core_mobile_base/simulation/SimulationControl2ASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2AS2RWD.hpp"

namespace romea
{

using SimulationCommand2AS2RWD = SimulationCommand2ASxxx;
using SimulationState2AS2RWD = SimulationState2ASxxx;

void toSimulation(
  const double & wheelbase,
  const double & frontTrack,
  const double & frontWheelRadius,
  const double & frontHubCarrierOffset,
  const double & rearTrack,
  const double & rearWheelRadius,
  const double & rearHubCarrierOffset,
  const HardwareCommand2AS2RWD & hardwareCommand,
  SimulationCommand2AS2RWD & simulationCommand);

void fromSimulation(
  const double & wheelbase,
  const double & front_track,
  const double & rearTrack,
  const SimulationState2AS2RWD & simulationState,
  HardwareState2AS2RWD & hardwareState);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2AS2RWD_HPP_
