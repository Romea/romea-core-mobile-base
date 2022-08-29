#ifndef romea_SimulationHardwareControl2AS4WD_hpp
#define romea_SimulationHardwareControl2AS4WD_hpp

#include "SimulationControl2ASxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2AS4WD.hpp"

namespace romea {

using SimulationCommand2AS4WD = SimulationCommand2ASxxx;
using SimulationState2AS4WD = SimulationState2ASxxx;


SimulationCommand2AS4WD toSimulationCommand2AS4WD(const double & wheelbase,
                                                  const double & frontTrack,
                                                  const double & rearTrack,
                                                  const HardwareCommand2AS4WD & hardwareCommand);

HardwareState2AS4WD toHardwareState2AS4WD(const double & wheelbase,
                                          const double & front_track,
                                          const double &rearTrack,
                                          const SimulationState2AS4WD & simulationState);


}//end romea

#endif
