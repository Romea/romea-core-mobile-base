#ifndef ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL1FAS2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL1FAS2RWD_HPP_

#include "romea_core_mobile_base/simulation/SimulationControl1FASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl1FAS2RWD.hpp"

namespace romea {

using SimulationCommand1FAS2RWD = SimulationCommand1FASxxx;
using SimulationState1FAS2RWD = SimulationState1FASxxx;

SimulationCommand1FAS2RWD toSimulationCommand1FAS2RWD(const double & wheelbase,
                                                      const double & frontTrack,
                                                      const double & frontHubCarrierOffset,
                                                      const double & frontWheelRadius,
                                                      const double & rearWheelRadius,
                                                      const HardwareCommand1FAS2RWD & hardwareCommand);

HardwareState1FAS2RWD toHardwareState1FAS2RWD(const double & wheelbase,
                                              const double & frontTrack,
                                              const SimulationState1FAS2RWD & simulationState);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL1FAS2RWD_HPP_
