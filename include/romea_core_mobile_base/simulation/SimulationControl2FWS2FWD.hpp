#ifndef romea_SimulationHardwareControl2FWS2FWD_hpp
#define romea_SimulationHardwareControl2FWS2FWD_hpp

#include "SimulationControl2FWSxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2FWS2FWD.hpp"

namespace romea {

using  SimulationCommand2FWS2FWD = SimulationCommand2FWSxxx;
using  SimulationState2FWS2FWD = SimulationState2FWSxxx;

SimulationCommand2FWS2FWD  toSimulationCommand2FWS2FWD(const double & wheelbase,
                                                       const double & frontTrack,
                                                       const double & rearTrack,
                                                       const double & frontWheelRadius,
                                                       const double & rearWheelRadius,
                                                       const double & frontHubCarrierOffset,
                                                       const double & rearHubCarrierOffset,
                                                       const HardwareCommand2FWS2FWD & hardwareCommand);

HardwareState2FWS2FWD toHardwareState2FWS2FWD (const SimulationState2FWS2FWD & simulationState);


}//end romea

#endif
