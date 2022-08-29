#ifndef romea_SimulationControl1FAS2FWD_hpp
#define romea_SimulationControl1FAS2FWD_hpp

#include "SimulationControl1FASxxx.hpp"
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


}//end romea

#endif
