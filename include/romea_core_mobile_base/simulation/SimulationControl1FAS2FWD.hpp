#ifndef ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL1FAS2FWD_HPP_
#define ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL1FAS2FWD_HPP_

#include "romea_core_mobile_base/simulation/SimulationControl1FASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl1FAS2FWD.hpp"

namespace romea {

using SimulationCommand1FAS2FWD = SimulationCommand1FASxxx;
using SimulationState1FAS2FWD = SimulationState1FASxxx;

SimulationCommand1FAS2FWD toSimulationCommand1FAS2FWD(const double & wheelbase,
                                                      const double & frontTrack,
                                                      const double & rearTrack,
                                                      const double & frontWheelRadius,
                                                      const double & rearWheelRadius,
                                                      const double & frontHubCarrierOffset,
                                                      const double & rearHubCarrierOffset,
                                                      const HardwareCommand1FAS2FWD & hardwareCommand);

HardwareState1FAS2FWD toHardwareState1FAS2FWD(const double & wheelbase,
                                              const double & frontTrack,
                                              const SimulationState1FAS2FWD & simulationState);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL1FAS2FWD_HPP_
