#ifndef romea_SimulationHardwareControl2FWS4WD_hpp
#define romea_SimulationHardwareControl2FWS4WD_hpp

#include "SimulationControl2FWSxxx.hpp"
#include "romea_core_mobile_base/hardware/HardwareControl2FWS4WD.hpp"

namespace romea {

using  SimulationCommand2FWS4WD = SimulationCommand2FWSxxx;
using  SimulationState2FWS4WD = SimulationState2FWSxxx;

//SimulationCommand2FWS4WD toSimulationCommand2FWS4WD(const double & wheelbase,
//                                                    const double & frontTrack,
//                                                    const HardwareCommand2FWS4WD & hardwareCommand);

//HardwareState2FWS4WD toHardwareState2FWS4WD(const SimulationState2FWS4WD & simulationState);


}//end romea

#endif
