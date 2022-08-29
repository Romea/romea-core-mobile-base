#ifndef romea_SimulationControl1FWS2RWD_hpp
#define romea_SimulationControl1FWS2RWD_hpp

#include "romea_core_mobile_base/hardware/HardwareControl1FWS2RWD.hpp"

namespace romea {

struct SimulationCommand1FWS2RWD
{
  SteeringAngleCommand frontWheelSteeringAngle;
  RotationalMotionCommand frontWheelSetPoint;
  RotationalMotionCommand rearLeftWheelSetPoint;
  RotationalMotionCommand rearRightWheelSetPoint;
};

struct SimulationState1FWS2RWD
{
  SteeringAngleState frontWheelSteeringAngle;
  RotationalMotionState frontWheelSpinMotion;
  RotationalMotionState rearLeftWHeelSpinMotion;
  RotationalMotionState rearRightWHeelSpinMotion;
};

SimulationCommand1FWS2RWD toSimulationCommand1FWS2RWD(const double & frontWheelRadius,
                                                      const double & rearWheelRadius,
                                                      const HardwareCommand1FWS2RWD & hardwareCommand);

HardwareState1FWS2RWD toHardwareState1FWS2RWD(const SimulationState1FWS2RWD & simulationState);


}//end romea
#endif
