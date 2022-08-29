#ifndef romea_SimulationHardwareControl1FASxxx_hpp
#define romea_SimulationHardwareControl1FASxxx_hpp

#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

namespace romea {

struct SimulationCommand1FASxxx
{
  SteeringAngleCommand frontAxleSteeringAngle;
  SteeringAngleCommand frontLeftWheelSteeringAngle;
  SteeringAngleCommand frontRightWheelSteeringAngle;

  RotationalMotionCommand frontLeftWheelSetPoint;
  RotationalMotionCommand frontRightWheelSetPoint;
  RotationalMotionCommand rearLeftWheelSetPoint;
  RotationalMotionCommand rearRightWheelSetPoint;
};

std::ostream & operator<<(std::ostream &os, const SimulationCommand1FASxxx & state);

struct SimulationState1FASxxx
{
  SteeringAngleState frontAxleSteeringAngle;
  SteeringAngleState frontLeftWheelSteeringAngle;
  SteeringAngleState frontRightWheelSteeringAngle;

  RotationalMotionState frontLeftWheelSpinMotion;
  RotationalMotionState frontRightWheelSpinMotion;
  RotationalMotionState rearLeftWheelSpinMotion;
  RotationalMotionState rearRightWheelSpinMotion;
};

std::ostream & operator<<(std::ostream &os, const SimulationState1FASxxx & state);

}//end romea

#endif
