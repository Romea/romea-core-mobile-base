#ifndef romea_SimulationHardwareControl2ASxxx_hpp
#define romea_SimulationHardwareControl2ASxxx_hpp

#include "romea_core_mobile_base/hardware/HardwareControl2AS4WD.hpp"

namespace romea {

struct SimulationCommand2ASxxx
{
  SteeringAngleCommand frontAxleSteeringAngle;
  SteeringAngleCommand frontLeftWheelSteeringAngle;
  SteeringAngleCommand frontRightWheelSteeringAngle;
  SteeringAngleCommand rearAxleSteeringAngle;
  SteeringAngleCommand rearLeftWheelSteeringAngle;
  SteeringAngleCommand rearRightWheelSteeringAngle;

  RotationalMotionCommand frontLeftWheelSpinningSetPoint;
  RotationalMotionCommand frontRightWheelSpinningSetPoint;
  RotationalMotionCommand rearLeftWheelSpinningSetPoint;
  RotationalMotionCommand rearRightWheelSpinningSetPoint;
};

struct SimulationState2ASxxx
{
  SteeringAngleState frontAxleSteeringAngle;
  SteeringAngleState frontLeftWheelSteeringAngle;
  SteeringAngleState frontRightWheelSteeringAngle;
  SteeringAngleState rearAxleSteeringAngle;
  SteeringAngleState rearLeftWheelSteeringAngle;
  SteeringAngleState rearRightWheelSteeringAngle;

  RotationalMotionState frontLeftWheelSpinningMotion;
  RotationalMotionState frontRightWheelSpinningMotion;
  RotationalMotionState rearLeftWheelSpinningMotion;
  RotationalMotionState rearRightWheelSpinningMotion;
};


}//end romea

#endif
