#include "romea_core_mobile_base/simulation/SimulationControl2TTD.hpp"



namespace romea
{

//-----------------------------------------------------------------------------
SimulationCommand2TTD toSimulationCommand2TTD(const double & sprocketWheelRadius,
                                              const double & rollerWheelRadius,
                                              const double & idlerWheelRadius,
                                              const HardwareCommand2TD & hardwareCommand)
{

  const double idler_ratio = sprocketWheelRadius/idlerWheelRadius;
  const double roller_ratio = sprocketWheelRadius/rollerWheelRadius;

  return {hardwareCommand.leftSprocketWheelSetPoint,
        hardwareCommand.rightSprocketWheelSetPoint,
        hardwareCommand.leftSprocketWheelSetPoint*idler_ratio,
        hardwareCommand.rightSprocketWheelSetPoint*idler_ratio,
        hardwareCommand.leftSprocketWheelSetPoint*roller_ratio,
        hardwareCommand.rightSprocketWheelSetPoint*roller_ratio,
        hardwareCommand.leftSprocketWheelSetPoint*roller_ratio,
        hardwareCommand.rightSprocketWheelSetPoint*roller_ratio};
}

//-----------------------------------------------------------------------------
HardwareState2TD toHardwareState2TTD(const double & sprocketWheelRadius,
                                     const double & rollerWheelRadius,
                                     const SimulationState2TTD & simulationState)
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
