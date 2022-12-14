#ifndef ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL2ASXXX_HPP_
#define ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL2ASXXX_HPP_

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


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_SIMULATION_SIMULATIONCONTROL2ASXXX_HPP_
