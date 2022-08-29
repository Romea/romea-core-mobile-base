#include "romea_core_mobile_base/simulation/SimulationControl2TTD.hpp"



namespace romea
{

//-----------------------------------------------------------------------------
void toSimulation(const double & sprocketWheelRadius,
                  const double & rollerWheelRadius,
                  const double & idlerWheelRadius,
                  const HardwareCommand2TD & hardwareCommand,
                  SimulationCommand2TTD & simulationCommand)
{

   const double idler_ratio = sprocketWheelRadius/idlerWheelRadius;
   const double roller_ratio = sprocketWheelRadius/rollerWheelRadius;

   simulationCommand.leftSprocketWheelSetPoint = hardwareCommand.leftSprocketWheelSetPoint;
   simulationCommand.rightSprocketWheelSetPoint = hardwareCommand.rightSprocketWheelSetPoint;

   simulationCommand.leftIdlerWheelSetPoint = hardwareCommand.leftSprocketWheelSetPoint*idler_ratio;
   simulationCommand.rightIdlerWheelSetPoint = hardwareCommand.rightSprocketWheelSetPoint*idler_ratio;

   simulationCommand.frontLeftRollerWheelSetPoint = hardwareCommand.leftSprocketWheelSetPoint*roller_ratio;
   simulationCommand.frontRightRollerWheelSetPoint = hardwareCommand.rightSprocketWheelSetPoint*roller_ratio;
   simulationCommand.rearLeftRollerWheelSetPoint = hardwareCommand.leftSprocketWheelSetPoint*roller_ratio;
   simulationCommand.rearRightRollerWheelSetPoint = hardwareCommand.rightSprocketWheelSetPoint*roller_ratio;
}

//-----------------------------------------------------------------------------
void fromSimulation(const double &sprocketWheelRadius,
                    const double &idlerWheelRadius,
                    const SimulationState2TTD & simulationState,
                    HardwareState2TD & hardwareState)
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
