#include "romea_core_mobile_base/simulation/SimulationControl2THD.hpp"
#include <cmath>

namespace  {

romea::RotationalMotionState toHardwareSprocketSpinMotion(const double & sprocketWheelRadius,
                                                          const double & idlerWheelRadius,
                                                          const double & trackThickness,
                                                          const romea::RotationalMotionState & sprocketWheelSpinMotion,
                                                          const romea::RotationalMotionState & frontIdlerWheelSpinMotion,
                                                          const romea::RotationalMotionState & rearIdlerWheelSpinMotion)
{
  const double sprocketWheelVirtualRadius =sprocketWheelRadius+trackThickness;
  const double idlerWheelVirtualRadius =idlerWheelRadius+trackThickness;

  const double ratio = idlerWheelVirtualRadius/sprocketWheelVirtualRadius;
  const double sprocketWheelLinearSpeed = sprocketWheelVirtualRadius*sprocketWheelSpinMotion.velocity;
  const double frontIdlerWheelLinearSpeed = idlerWheelVirtualRadius*frontIdlerWheelSpinMotion.velocity;
  const double rearIdlerWheelLinearSpeed = idlerWheelVirtualRadius*rearIdlerWheelSpinMotion.velocity;

  romea::RotationalMotionState output;
  output.position= sprocketWheelSpinMotion.position;

  if(std::signbit(frontIdlerWheelLinearSpeed)!=std::signbit(rearIdlerWheelLinearSpeed))
  {
    output.velocity=0;
    output.torque=0;
  }
  else if(std::abs(frontIdlerWheelLinearSpeed)<std::abs(rearIdlerWheelLinearSpeed))
  {
    output.velocity =frontIdlerWheelSpinMotion.velocity*ratio;
    output.torque =frontIdlerWheelSpinMotion.torque*ratio;
  }
  else
  {
    output.velocity = rearIdlerWheelSpinMotion.velocity*ratio;
    output.torque = rearIdlerWheelSpinMotion.torque*ratio;
  }
  return output;
}


}

namespace romea
{

//-----------------------------------------------------------------------------
SimulationCommand2THD toSimulationCommand2THD(const double & sprocketWheelRadius,
                                              const double & idlerWheelRadius,
                                              const double & trackThickness,
                                              const HardwareCommand2TD & hardwareCommand)
{

  const double ratio = (sprocketWheelRadius+trackThickness)/(idlerWheelRadius+trackThickness);

  return{hardwareCommand.leftSprocketWheelSetPoint,
        hardwareCommand.rightSprocketWheelSetPoint,
        hardwareCommand.leftSprocketWheelSetPoint*ratio,
        hardwareCommand.rightSprocketWheelSetPoint*ratio,
        hardwareCommand.leftSprocketWheelSetPoint*ratio,
        hardwareCommand.rightSprocketWheelSetPoint*ratio};
}

//-----------------------------------------------------------------------------
HardwareState2TD toHardwareState2TD(const double &sprocketWheelRadius,
                                    const double &idlerWheelRadius,
                                    const double & trackThickness,
                                    const SimulationState2THD & simulationState)
{
  return {toHardwareSprocketSpinMotion(sprocketWheelRadius,
                                       idlerWheelRadius,
                                       trackThickness,
                                       simulationState.leftSprocketWheelSpinMotion,
                                       simulationState.frontLeftIdlerWheelSpinMotion,
                                       simulationState.rearLeftIdlerWheelSpinMotion),
        toHardwareSprocketSpinMotion(sprocketWheelRadius,
                                     idlerWheelRadius,
                                     trackThickness,
                                     simulationState.rightSprocketWheelSpinMotion,
                                     simulationState.frontRightIdlerWheelSpinMotion,
                                     simulationState.rearRightIdlerWheelSpinMotion)};
}


}//end romea
