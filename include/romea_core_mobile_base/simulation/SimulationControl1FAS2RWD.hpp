#ifndef romea_SimulationControl1FAS2RWD_hpp
#define romea_SimulationControl1FAS2RWD_hpp

#include "SimulationControl1FASxxx.hpp"
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


}//end romea

#endif
