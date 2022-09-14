// gtest
#include <gtest/gtest.h>

//romea
#include "romea_core_mobile_base/simulation/SimulationControl2TTD.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"

romea::HardwareCommand2TD toHardwareCommand2TD(const double & sprocketWheelRadius,
                                               const double & trackThickness,
                                               const romea::OdometryFrame2TD & odometryFrame)
{
  return {odometryFrame.leftTrackLinearSpeed/(sprocketWheelRadius+trackThickness),
        odometryFrame.rightTrackLinearSpeed/(sprocketWheelRadius+trackThickness)};
}

//romea::HardwareCommand2FWS4WD toHardwareCommand2FWS4WD(const double & frontWheelRadius,
//                                                       const double & rearWheelRadius,
//                                                       const romea::OdometryFrame2TD & odometryFrame)
//{
//  return {odometryFrame.frontLeftWheelSteeringAngle,
//        odometryFrame.frontRightWheelSteeringAngle,
//        odometryFrame.frontLeftWheelLinearSpeed/frontWheelRadius,
//        odometryFrame.frontRightWheelLinearSpeed/frontWheelRadius,
//        odometryFrame.rearLeftWheelLinearSpeed/rearWheelRadius,
//        odometryFrame.rearRightWheelLinearSpeed/rearWheelRadius};
//}

class TestSimulation2TTD : public ::testing::Test
{

public :


  TestSimulation2TTD(){}

  virtual void SetUp()override
  {
    sprocketWheelRadius = 0.8;
    idlerWheelRadius = 0.3;
    rollerWheelRadius = 0.2;
    trackThickness=0.1;
    parameters.wheelTrack=1.5;

    command.longitudinalSpeed= 1.;
    command.angularSpeed = 0.6;

    romea::OdometryFrame2TD odometryCommand;
    romea::forwardKinematic(parameters,command,odometryCommand);
    hardwareCommand2TD = toHardwareCommand2TD(sprocketWheelRadius,
                                              trackThickness,
                                              odometryCommand);

    std::cout << "hardwareCommand2TD " <<odometryCommand << std::endl;

    simulationCommand2TTD = toSimulationCommand2TTD(sprocketWheelRadius,
                                                    rollerWheelRadius,
                                                    idlerWheelRadius,
                                                    trackThickness,
                                                    hardwareCommand2TD);

    std::cout << "simulationCommand2TTD " <<simulationCommand2TTD << std::endl;

  }

  double sprocketWheelRadius;
  double rollerWheelRadius;
  double idlerWheelRadius;
  double trackThickness;
  romea::SkidSteeringKinematic::Parameters parameters;

  romea::SkidSteeringCommand command;
  romea::HardwareCommand2TD hardwareCommand2TD;
  romea::SimulationCommand2TTD simulationCommand2TTD;

};


TEST_F(TestSimulation2TTD,toSimulation)
{
  EXPECT_DOUBLE_EQ(simulationCommand2TTD.leftSprocketWheelSpinningSetPoint,
                   hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_DOUBLE_EQ(simulationCommand2TTD.rightSprocketWheelSpinningSetPoint,
                   hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

  EXPECT_GT(simulationCommand2TTD.leftIdlerWheelSpinningSetPoint,
            hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_GT(simulationCommand2TTD.rightIdlerWheelSpinningSetPoint,
            hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

  EXPECT_GT(simulationCommand2TTD.frontLeftRollerWheelSpinningSetPoint,
            hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_GT(simulationCommand2TTD.frontRightRollerWheelSpinningSetPoint,
            hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

  EXPECT_GT(simulationCommand2TTD.rearLeftRollerWheelSpinningSetPoint,
            hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_GT(simulationCommand2TTD.rearRightRollerWheelSpinningSetPoint,
            hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

}


TEST_F(TestSimulation2TTD,toHardware)
{


  romea::SimulationState2TTD simulationState;
  simulationState.leftSprocketWheelSpinningMotion.velocity = simulationCommand2TTD.leftSprocketWheelSpinningSetPoint*2;
  simulationState.rightSprocketWheelSpinningMotion.velocity = simulationCommand2TTD.rightSprocketWheelSpinningSetPoint*2;
  simulationState.leftIdlerWheelSpinningMotion.velocity = simulationCommand2TTD.leftIdlerWheelSpinningSetPoint*2;
  simulationState.rightIdlerWheelSpinningMotion.velocity = simulationCommand2TTD.rightIdlerWheelSpinningSetPoint*2;
  simulationState.frontLeftRollerWheelSpinningMotion.velocity = simulationCommand2TTD.frontLeftRollerWheelSpinningSetPoint;
  simulationState.rearLeftRollerWheelSpinningMotion.velocity = simulationCommand2TTD.rearLeftRollerWheelSpinningSetPoint+0.1;
  simulationState.frontRightRollerWheelSpinningMotion.velocity = simulationCommand2TTD.frontRightRollerWheelSpinningSetPoint+0.1;
  simulationState.rearRightRollerWheelSpinningMotion.velocity = simulationCommand2TTD.rearRightRollerWheelSpinningSetPoint;


  auto hardwareState2TD =romea::toHardwareState2TTD(sprocketWheelRadius,
                                                    rollerWheelRadius,
                                                    trackThickness,
                                                    simulationState);

  EXPECT_DOUBLE_EQ(hardwareState2TD.leftSprocketWheelSpinningMotion.velocity,
                   hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(hardwareState2TD.rightSprocketWheelSpinningMotion.velocity,
                   hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
