#ifndef romea_SimulationHardwareControl2AS2RWD_hpp
#define romea_SimulationHardwareControl2AS2RWD_hpp

#include "SimulationControl2ASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2AS2RWD.hpp"

namespace romea {

using SimulationCommand2AS2RWD = SimulationCommand2ASxxx;
using SimulationState2AS2RWD = SimulationState2ASxxx;

void toSimulation(const double & wheelbase,
                  const double & frontTrack,
                  const double & frontWheelRadius,
                  const double & frontHubCarrierOffset,
                  const double & rearTrack,
                  const double & rearWheelRadius,
                  const double & rearHubCarrierOffset,
                  const HardwareCommand2AS2RWD & hardwareCommand,
                  SimulationCommand2AS2RWD & simulationCommand);

void fromSimulation(const double & wheelbase,
                    const double & front_track,
                    const double &rearTrack,
                    const SimulationState2AS2RWD & simulationState,
                    HardwareState2AS2RWD & hardwareState);


}//end romea

#endif
