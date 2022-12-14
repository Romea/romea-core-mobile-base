#ifndef ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL1FAS4WD_HPP_
#define ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL1FAS4WD_HPP_

#include "romea_core_mobile_base/simulation/SimulationControl1FASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl1FAS4WD.hpp"

namespace romea {

using SimulationCommand1FAS4WD = SimulationCommand1FASxxx;
using SimulationState1FAS4WD = SimulationState1FASxxx;

void toSimulation(const double & wheelbase,
                  const double & frontTrack,
                  const HardwareCommand1FAS4WD & hardwareCommand,
                  SimulationCommand1FAS4WD & simulationCommand);

void fromSimulation(const double & wheelbase,
                    const double & frontTrack,
                    const SimulationState1FAS4WD & simulationState,
                    HardwareState1FAS4WD & hardwareState);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL1FAS4WD_HPP_
