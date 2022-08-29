#include "romea_core_mobile_base/simulation/SimulationControl2THD.hpp"



namespace romea
{

//-----------------------------------------------------------------------------
SimulationCommand2THD toSimulationCommand2THD(const double & sprocketWheelRadius,
                                              const double & idlerWheelRadius,
                                              const HardwareCommand2TD & hardwareCommand)
{

  const double ratio = sprocketWheelRadius/idlerWheelRadius;

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
                                    const SimulationState2THD & simulationState)
{
  //  hardwareState.leftSprocketWheelSpinMotion.position =
  //      simulationState.leftSprocketWheelSpinMotion.position;
  //  hardwareState.rightSprocketWheelSpinMotion.position =
  //      simulationState.rightSprocketWheelSpinMotion.position;

  //  if(simulationState.frontLeftIdlerWheelSpinMotion.velocity<
  //     simulationState.rearLeftIdlerWheelSpinMotion.velocity)
  //  {
  //    hardwareState.leftSprocketWheelSpinMotion.velocity =
  //        simulationState.frontLeftIdlerWheelSpinMotion.velocity;
  //    hardwareState.leftSprocketWheelSpinMotion.torque =
  //        simulationState.frontLeftIdlerWheelSpinMotion.torque;

  //    hardwareState.rightSprocketWheelSpinMotion.velocity =
  //        simulationState.rightSprocketWheelSpinMotion.velocity;
  //    hardwareState.rightSprocketWheelSpinMotion.torque =
  //        simulationState.rightSprocketWheelSpinMotion.torque;

  //  }
  //  else if (simulationState.leftSprocketWheelSpinMotion.velocity>
  //           simulationState.leftIdlerWheelSpinMotion.velocity)
  //  {
  //    hardwareState.leftSprocketWheelSpinMotion.velocity =
  //        simulationState.leftIdlerWheelSpinMotion.velocity;
  //    hardwareState.leftSprocketWheelSpinMotion.torque =
  //        simulationState.leftIdlerWheelSpinMotion.torque;

  //    hardwareState.rightSprocketWheelSpinMotion.velocity =
  //        simulationState.rightIdlerWheelSpinMotion.velocity;
  //    hardwareState.rightSprocketWheelSpinMotion.torque =
  //        simulationState.rightIdlerWheelSpinMotion.torque;
  //  }
  //  else
  //  {
  //    hardwareState.leftSpinMotion.velocity=0;
  //    hardwareState.leftSpinMotion.torque=0;
  //    hardwareState.rightSpinMotion.velocity=0;
  //    hardwareState.rightSpinMotion.torque=0;
  //  }
}


}//end romea
