#ifndef romea_SimulationHardwareControl2AS2FWD_hpp
#define romea_SimulationHardwareControl2AS2FWD_hpp

#include "SimulationControl2ASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2AS2FWD.hpp"

namespace romea {

using SimulationCommand2AS2FWD = SimulationCommand2ASxxx;
using SimulationState2AS2FWD = SimulationState2ASxxx;

//TODO
//void toSimulation(const double & wheelbase,
//                  const double & frontTrack,
//                  const double & rearTrack,
//                  const HardwareCommand2AS4WD & hardwareCommand,
//                  SimulationHardwareCommand2AS4WD & simulationCommand);

//void fromSimulation(const double & wheelbase,
//                    const double & front_track,
//                    const double &rearTrack,
//                    const SimulationHardwareState2AS4WD & simulationState,
//                    HardwareState2AS4WD & hardwareState);


}//end romea

#endif
