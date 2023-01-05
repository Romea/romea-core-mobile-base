// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2AS4WD_HPP_
#define ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2AS4WD_HPP_

#include "romea_core_mobile_base/simulation/SimulationControl2ASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2AS4WD.hpp"

namespace romea
{

using SimulationCommand2AS4WD = SimulationCommand2ASxxx;
using SimulationState2AS4WD = SimulationState2ASxxx;

SimulationCommand2AS4WD toSimulationCommand2AS4WD(
  const double & wheelbase,
  const double & frontTrack,
  const double & rearTrack,
  const HardwareCommand2AS4WD & hardwareCommand);

HardwareState2AS4WD toHardwareState2AS4WD(
  const double & wheelbase,
  const double & front_track,
  const double & rearTrack,
  const SimulationState2AS4WD & simulationState);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__SIMULATION__SIMULATIONCONTROL2AS4WD_HPP_
