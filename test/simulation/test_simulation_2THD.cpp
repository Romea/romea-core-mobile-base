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

    std::cout << odometryCommand << std::endl;

    simulationCommand2THD = toSimulationCommand2THD(sprocketWheelRadius,
                                                    idlerWheelRadius,
                                                    trackThickness,
                                                    hardwareCommand2TD);

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
  std::cout << simulationCommand2THD.leftSprocketWheelSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rightSprocketWheelSetPoint <<std::endl;
  std::cout << simulationCommand2THD.frontLeftIdlerWheelSetPoint <<std::endl;
  std::cout << simulationCommand2THD.frontRightIdlerWheelSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rearLeftIdlerWheelSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rearRightIdlerWheelSetPoint <<std::endl;

  EXPECT_DOUBLE_EQ(simulationCommand2THD.leftSprocketWheelSetPoint,
                   hardwareCommand2TD.leftSprocketWheelSetPoint);

  EXPECT_DOUBLE_EQ(simulationCommand2THD.rightSprocketWheelSetPoint,
                   hardwareCommand2TD.rightSprocketWheelSetPoint);

  EXPECT_GT(simulationCommand2THD.frontLeftIdlerWheelSetPoint,
            hardwareCommand2TD.leftSprocketWheelSetPoint);

  EXPECT_GT(simulationCommand2THD.frontRightIdlerWheelSetPoint,
            hardwareCommand2TD.rightSprocketWheelSetPoint);

  EXPECT_GT(simulationCommand2THD.rearLeftIdlerWheelSetPoint,
            hardwareCommand2TD.leftSprocketWheelSetPoint);

  EXPECT_GT(simulationCommand2THD.rearRightIdlerWheelSetPoint,
            hardwareCommand2TD.rightSprocketWheelSetPoint);

}


TEST_F(TestSimulation2THD,toHardware)
{


  romea::SimulationState2THD simulationState;
  simulationState.leftSprocketWheelSpinMotion.velocity = simulationCommand2THD.leftSprocketWheelSetPoint*2;
  simulationState.rightSprocketWheelSpinMotion.velocity = simulationCommand2THD.rightSprocketWheelSetPoint*2;
  simulationState.frontLeftIdlerWheelSpinMotion.velocity = simulationCommand2THD.frontLeftIdlerWheelSetPoint;
  simulationState.rearLeftIdlerWheelSpinMotion.velocity = simulationCommand2THD.rearLeftIdlerWheelSetPoint+0.1;
  simulationState.frontRightIdlerWheelSpinMotion.velocity = simulationCommand2THD.frontRightIdlerWheelSetPoint+0.1;
  simulationState.rearRightIdlerWheelSpinMotion.velocity = simulationCommand2THD.rearRightIdlerWheelSetPoint;


  std::cout << " simulation command "<< std::endl;
  std::cout << simulationCommand2THD.leftSprocketWheelSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rightSprocketWheelSetPoint <<std::endl;
  std::cout << simulationCommand2THD.frontLeftIdlerWheelSetPoint <<std::endl;
  std::cout << simulationCommand2THD.frontRightIdlerWheelSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rearLeftIdlerWheelSetPoint <<std::endl;
  std::cout << simulationCommand2THD.rearRightIdlerWheelSetPoint <<std::endl;

  auto hardwareState2TD =romea::toHardwareState2TD(sprocketWheelRadius,
                                                   idlerWheelRadius,
                                                   trackThickness,
                                                   simulationState);

  EXPECT_DOUBLE_EQ(hardwareState2TD.leftSprocketWheelSpinMotion.velocity,
                   hardwareCommand2TD.leftSprocketWheelSetPoint);
  EXPECT_DOUBLE_EQ(hardwareState2TD.rightSprocketWheelSpinMotion.velocity,
                   hardwareCommand2TD.rightSprocketWheelSetPoint);

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
