#include "romea_core_mobile_base/simulation/SimulationControl2TD.hpp"
#include <cmath>

namespace  {

romea::RotationalMotionState toHardwareSprocketSpinMotion(const double & sprocketWheelRadius,
                                                          const double & idlerWheelRadius,
                                                          const double & trackThickness,
                                                          const romea::RotationalMotionState & sprocketWheelSpinMotion,
                                                          const romea::RotationalMotionState & idlerWheelSpinMotion)
{
  const double sprocketWheelVirtualRadius =sprocketWheelRadius+trackThickness;
  const double idlerWheelVirtualRadius =idlerWheelRadius+trackThickness;

  const double ratio = idlerWheelVirtualRadius/sprocketWheelVirtualRadius;
  const double sprocketWheelLinearSpeed = sprocketWheelVirtualRadius*sprocketWheelSpinMotion.velocity;
  const double idlerWheelLinearSpeed = idlerWheelVirtualRadius*idlerWheelSpinMotion.velocity;

  romea::RotationalMotionState output;
  output.position= sprocketWheelSpinMotion.position;

  if(std::signbit(sprocketWheelLinearSpeed)!=std::signbit(idlerWheelLinearSpeed))
  {
    output.velocity=0;
    output.torque=0;
  }
  else if(std::abs(sprocketWheelLinearSpeed)<std::abs(idlerWheelLinearSpeed))
  {
    output.velocity =sprocketWheelSpinMotion.velocity;
    output.torque =sprocketWheelSpinMotion.torque;
  }
  else
  {
    output.velocity =idlerWheelSpinMotion.velocity*ratio;
    output.torque =idlerWheelSpinMotion.torque*ratio;
  }
  return output;
}

}

namespace romea
{


//-----------------------------------------------------------------------------
SimulationCommand2TD toSimulationCommand2TD(const double & sprocketWheelRadius,
                                            const double & idlerWheelRadius,
                                            const double & trackThickness,
                                            const HardwareCommand2TD & hardwareCommand)
{

  const double ratio = (sprocketWheelRadius+trackThickness)/(idlerWheelRadius+trackThickness);

  return {hardwareCommand.leftSprocketWheelSetPoint,
        hardwareCommand.rightSprocketWheelSetPoint,
        hardwareCommand.leftSprocketWheelSetPoint*ratio,
        hardwareCommand.rightSprocketWheelSetPoint*ratio};

}

//-----------------------------------------------------------------------------
HardwareState2TD toHardwareState2TD(const double & sprocketWheelRadius,
                                    const double & idlerWheelRadius,
                                    const double & trackThickness,
                                    const SimulationState2TD & simulationState)
{
  return {toHardwareSprocketSpinMotion(sprocketWheelRadius,
                                       idlerWheelRadius,
                                       trackThickness,
                                       simulationState.leftSprocketWheelSpinMotion,
                                       simulationState.leftIdlerWheelSpinMotion),
        toHardwareSprocketSpinMotion(sprocketWheelRadius,
                                     idlerWheelRadius,
                                     trackThickness,
                                     simulationState.rightSprocketWheelSpinMotion,
                                     simulationState.rightIdlerWheelSpinMotion)};
}

}//end romea
