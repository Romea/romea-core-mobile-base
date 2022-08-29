// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_core_mobile_base/simulation/SimulationControl2TD.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"

romea::HardwareCommand2TD toHardwareCommand2TD(const double & sprocketWheelRadius,
                                               const double & trackWidth,
                                               const romea::OdometryFrame2TD & odometryFrame)
{
  return {odometryFrame.leftTrackLinearSpeed/(sprocketWheelRadius+trackWidth),
        odometryFrame.rightTrackLinearSpeed/(sprocketWheelRadius+trackWidth)};
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

class TestSimulation2FWS2RWD : public ::testing::Test
{

public :


  TestSimulation2FWS2RWD(){}

  virtual void SetUp()override
  {
    sprocketWheelRadius = 0.8;
    idlerWheelRadius = 0.3;
    trackWidth=0.1;
    parameters.wheelTrack=1.5;

    command.longitudinalSpeed= 1.;
    command.angularSpeed = 0.6;

    romea::OdometryFrame2TD odometryCommand;
    romea::forwardKinematic(parameters,command,odometryCommand);
    hardwareCommand2TD = toHardwareCommand2TD(sprocketWheelRadius,
                                              trackWidth,
                                              odometryCommand);

    std::cout << odometryCommand << std::endl;

    simulationCommand2TD = toSimulationCommand2TD(sprocketWheelRadius,
                                                  idlerWheelRadius,
                                                  trackWidth,
                                                  hardwareCommand2TD);

  }

  double sprocketWheelRadius;
  double idlerWheelRadius;
  double trackWidth;
  romea::SkidSteeringKinematic::Parameters parameters;

  romea::SkidSteeringCommand command;
  romea::HardwareCommand2TD hardwareCommand2TD;
  romea::SimulationCommand2TD simulationCommand2TD;

};


TEST_F(TestSimulation2FWS2RWD,toSimulation)
{
  std::cout << " simulation command "<< std::endl;
  std::cout << simulationCommand2TD.leftSprocketWheelSetPoint <<std::endl;
  std::cout << simulationCommand2TD.rightSprocketWheelSetPoint <<std::endl;
  std::cout << simulationCommand2TD.leftIdlerWheelSetPoint <<std::endl;
  std::cout << simulationCommand2TD.rightIdlerWheelSetPoint <<std::endl;

   EXPECT_DOUBLE_EQ(simulationCommand2TD.leftSprocketWheelSetPoint,
                    hardwareCommand2TD.leftSprocketWheelSetPoint);

   EXPECT_DOUBLE_EQ(simulationCommand2TD.rightSprocketWheelSetPoint,
                    hardwareCommand2TD.rightSprocketWheelSetPoint);

   EXPECT_GT(simulationCommand2TD.leftIdlerWheelSetPoint,
             hardwareCommand2TD.leftSprocketWheelSetPoint);

   EXPECT_GT(simulationCommand2TD.rightIdlerWheelSetPoint,
             hardwareCommand2TD.rightSprocketWheelSetPoint);

}


TEST_F(TestSimulation2FWS2RWD,toHardware)
{


  romea::SimulationState2TD simulationState;
  simulationState.leftSprocketWheelSpinMotion.velocity = simulationCommand2TD.leftSprocketWheelSetPoint;
  simulationState.leftIdlerWheelSpinMotion.velocity = simulationCommand2TD.leftIdlerWheelSetPoint+0.1;
  simulationState.rightSprocketWheelSpinMotion.velocity = simulationCommand2TD.rightSprocketWheelSetPoint+0.1;
  simulationState.rightIdlerWheelSpinMotion.velocity = simulationCommand2TD.rightIdlerWheelSetPoint;


  std::cout << " simulation command "<< std::endl;
  std::cout << simulationCommand2TD.leftSprocketWheelSetPoint <<std::endl;
  std::cout << simulationCommand2TD.rightSprocketWheelSetPoint <<std::endl;
  std::cout << simulationCommand2TD.leftIdlerWheelSetPoint <<std::endl;
  std::cout << simulationCommand2TD.rightIdlerWheelSetPoint <<std::endl;

  auto hardwareState2TD =romea::toHardwareState2TD(sprocketWheelRadius,
                                                   idlerWheelRadius,
                                                   trackWidth,
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
