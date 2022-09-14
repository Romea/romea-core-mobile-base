#ifndef romea_SimulationHardwareControl2AS2FWD_hpp
#define romea_SimulationHardwareControl2AS2FWD_hpp

#include "SimulationControl2ASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2AS2FWD.hpp"

namespace romea {

using SimulationCommand2AS2FWD = SimulationCommand2ASxxx;
using SimulationState2AS2FWD = SimulationState2ASxxx;

void toSimulation(const double & wheelbase,
                  const double & frontTrack,
                  const double & frontWheelRadius,
                  const double & frontHubCarrierOffset,
                  const double & rearTrack,
                  const double & rearWheelRadius,
                  const double & rearHubCarrierOffset,
                  const HardwareCommand2AS2FWD & hardwareCommand,
                  SimulationCommand2AS2FWD & simulationCommand);

void fromSimulation(const double & wheelbase,
                    const double & front_track,
                    const double &rearTrack,
                    const SimulationState2AS2FWD & simulationState,
                    HardwareState2AS2FWD & hardwareState);


}//end romea

#endif
