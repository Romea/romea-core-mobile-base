// gtest
#include <gtest/gtest.h>

//romea
#include "romea_core_mobile_base/simulation/SimulationControl2THD.hpp"
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

class TestSimulation2THD : public ::testing::Test
{

public :


  TestSimulation2THD(){}

  virtual void SetUp()override
  {
    sprocketWheelRadius = 0.8;
    idlerWheelRadius = 0.3;
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

    simulationCommand2THD = toSimulationCommand2THD(sprocketWheelRadius,
                                                    idlerWheelRadius,
                                                    trackThickness,
                                                    hardwareCommand2TD);

    std::cout << "simulationCommand2THD " <<simulationCommand2THD << std::endl;


  }

  double sprocketWheelRadius;
  double idlerWheelRadius;
  double trackThickness;
  romea::SkidSteeringKinematic::Parameters parameters;

  romea::SkidSteeringCommand command;
  romea::HardwareCommand2TD hardwareCommand2TD;
  romea::SimulationCommand2THD simulationCommand2THD;

};


TEST_F(TestSimulation2THD,toSimulation)
{
  std::cout << " simulation command "<< std::endl;
  std::cout << simulationCommand2THD.leftSprocketWheelSpinningSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rightSprocketWheelSpinningSetPoint <<std::endl;
  std::cout << simulationCommand2THD.frontLeftIdlerWheelSpinningSetPoint <<std::endl;
  std::cout << simulationCommand2THD.frontRightIdlerWheelSpinningSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rearLeftIdlerWheelSpinningSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rearRightIdlerWheelSpinningSetPoint <<std::endl;

  EXPECT_DOUBLE_EQ(simulationCommand2THD.leftSprocketWheelSpinningSetPoint,
                   hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_DOUBLE_EQ(simulationCommand2THD.rightSprocketWheelSpinningSetPoint,
                   hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

  EXPECT_GT(simulationCommand2THD.frontLeftIdlerWheelSpinningSetPoint,
            hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_GT(simulationCommand2THD.frontRightIdlerWheelSpinningSetPoint,
            hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

  EXPECT_GT(simulationCommand2THD.rearLeftIdlerWheelSpinningSetPoint,
            hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_GT(simulationCommand2THD.rearRightIdlerWheelSpinningSetPoint,
            hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

}


TEST_F(TestSimulation2THD,toHardware)
{


  romea::SimulationState2THD simulationState;
  simulationState.leftSprocketWheelSpinningMotion.velocity = simulationCommand2THD.leftSprocketWheelSpinningSetPoint*2;
  simulationState.rightSprocketWheelSpinningMotion.velocity = simulationCommand2THD.rightSprocketWheelSpinningSetPoint*2;
  simulationState.frontLeftIdlerWheelSpinningMotion.velocity = simulationCommand2THD.frontLeftIdlerWheelSpinningSetPoint;
  simulationState.rearLeftIdlerWheelSpinningMotion.velocity = simulationCommand2THD.rearLeftIdlerWheelSpinningSetPoint+0.1;
  simulationState.frontRightIdlerWheelSpinningMotion.velocity = simulationCommand2THD.frontRightIdlerWheelSpinningSetPoint+0.1;
  simulationState.rearRightIdlerWheelSpinningMotion.velocity = simulationCommand2THD.rearRightIdlerWheelSpinningSetPoint;


  std::cout << " simulation command "<< std::endl;
  std::cout << simulationCommand2THD.leftSprocketWheelSpinningSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rightSprocketWheelSpinningSetPoint <<std::endl;
  std::cout << simulationCommand2THD.frontLeftIdlerWheelSpinningSetPoint <<std::endl;
  std::cout << simulationCommand2THD.frontRightIdlerWheelSpinningSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rearLeftIdlerWheelSpinningSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rearRightIdlerWheelSpinningSetPoint <<std::endl;

  auto hardwareState2TD =romea::toHardwareState2TD(sprocketWheelRadius,
                                                   idlerWheelRadius,
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