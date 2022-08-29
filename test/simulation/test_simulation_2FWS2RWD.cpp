// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_core_mobile_base/simulation/SimulationControl2FWS2RWD.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"

romea::HardwareCommand2FWS2RWD toHardwareCommand2FWS2RWD(const double & rearWheelRadius,
                                                         const romea::OdometryFrame2FWS2RWD & odometryFrame)
{
  return {odometryFrame.frontLeftWheelSteeringAngle,
        odometryFrame.frontRightWheelSteeringAngle,
        odometryFrame.rearLeftWheelLinearSpeed/rearWheelRadius,
        odometryFrame.rearRightWheelLinearSpeed/rearWheelRadius};
}

romea::HardwareCommand2FWS4WD toHardwareCommand2FWS4WD(const double & frontWheelRadius,
                                                       const double & rearWheelRadius,
                                                       const romea::OdometryFrame2FWS4WD & odometryFrame)
{
  return {odometryFrame.frontLeftWheelSteeringAngle,
        odometryFrame.frontRightWheelSteeringAngle,
        odometryFrame.frontLeftWheelLinearSpeed/frontWheelRadius,
        odometryFrame.frontRightWheelLinearSpeed/frontWheelRadius,
        odometryFrame.rearLeftWheelLinearSpeed/rearWheelRadius,
        odometryFrame.rearRightWheelLinearSpeed/rearWheelRadius};
}

class TestSimulation2FWS2RWD : public ::testing::Test
{

public :


  TestSimulation2FWS2RWD(){}

  virtual void SetUp()override
  {
    frontWheelRadius = 0.5;
    rearWheelRadius = 0.3;
    parameters.frontWheelBase= 0.4;
    parameters.rearWheelBase= 0.8;
    parameters.frontWheelTrack=1.4;
    parameters.rearWheelTrack=1.;
    parameters.frontHubCarrierOffset=0.1;
    parameters.rearHubCarrierOffset=0.05;

    command.longitudinalSpeed= 1.;
    command.steeringAngle = -0.5;

    romea::OdometryFrame2FWS2RWD odometryCommand;
    romea::forwardKinematic(parameters,command,odometryCommand);
    hardwareCommand2FWS2RWD = toHardwareCommand2FWS2RWD(rearWheelRadius,
                                                        odometryCommand);

    std::cout << odometryCommand << std::endl;

    simulationCommand2FWS2RWD = toSimulationCommand2FWS2RWD(parameters.frontWheelBase+
                                                            parameters.rearWheelBase,
                                                            parameters.frontWheelTrack,
                                                            parameters.frontHubCarrierOffset,
                                                            frontWheelRadius,
                                                            rearWheelRadius,
                                                            hardwareCommand2FWS2RWD);

  }

  double frontWheelRadius;
  double rearWheelRadius;
  romea::TwoWheelSteeringKinematic::Parameters parameters;

  romea::OneAxleSteeringCommand command;
  romea::HardwareCommand2FWS2RWD hardwareCommand2FWS2RWD;
  romea::SimulationCommand2FWS2RWD simulationCommand2FWS2RWD;

};


TEST_F(TestSimulation2FWS2RWD,toSimulation)
{

  romea::OdometryFrame2FWS4WD odometryCommand2FWS4WD;
  romea::forwardKinematic(parameters,command,odometryCommand2FWS4WD);
  auto hardwareCommand2FWS4WD = toHardwareCommand2FWS4WD(frontWheelRadius,
                                                         rearWheelRadius,
                                                         odometryCommand2FWS4WD);

  std::cout << odometryCommand2FWS4WD << std::endl;
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2RWD.frontLeftWheelSetPoint,
                   hardwareCommand2FWS4WD.frontLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2RWD.frontRightWheelSetPoint,
                   hardwareCommand2FWS4WD.frontRightWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2RWD.frontLeftWheelSteeringAngle,
                   hardwareCommand2FWS4WD.frontLeftWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2RWD.frontRightWheelSteeringAngle,
                   hardwareCommand2FWS2RWD.frontRightWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2RWD.frontLeftWheelSetPoint,
                   hardwareCommand2FWS4WD.frontLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2RWD.frontRightWheelSetPoint,
                   hardwareCommand2FWS4WD.frontRightWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2RWD.rearLeftWheelSetPoint,
                   hardwareCommand2FWS4WD.rearLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2RWD.rearRightWheelSetPoint,
                   hardwareCommand2FWS4WD.rearRightWheelSetPoint);
}


TEST_F(TestSimulation2FWS2RWD,toHardware)
{

  romea::SimulationState2FWSxxx simulationState;
  simulationState.frontLeftWheelSteeringAngle = simulationCommand2FWS2RWD.frontLeftWheelSteeringAngle;
  simulationState.frontRightWheelSteeringAngle = simulationCommand2FWS2RWD.frontRightWheelSteeringAngle;
  simulationState.frontLeftWheelSpinMotion.velocity = simulationCommand2FWS2RWD.frontLeftWheelSetPoint;
  simulationState.frontRightWheelSpinMotion.velocity = simulationCommand2FWS2RWD.frontRightWheelSetPoint;
  simulationState.rearLeftWheelSpinMotion.velocity = simulationCommand2FWS2RWD.rearLeftWheelSetPoint;
  simulationState.rearRightWheelSpinMotion.velocity = simulationCommand2FWS2RWD.rearRightWheelSetPoint;

  auto hardwareState2FWS2RWD =romea::toHardwareState2FWS2RWD(simulationState);

  EXPECT_DOUBLE_EQ(hardwareState2FWS2RWD.frontLeftWheelSteeringAngle,
                   hardwareCommand2FWS2RWD.frontLeftWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(hardwareState2FWS2RWD.frontRightWheelSteeringAngle,
                   hardwareCommand2FWS2RWD.frontRightWheelSteeringAngle);

  EXPECT_DOUBLE_EQ(hardwareState2FWS2RWD.rearLeftWheelSpinMotion.velocity,
                   hardwareCommand2FWS2RWD.rearLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(hardwareState2FWS2RWD.rearRightWheelSpinMotion.velocity,
                   hardwareCommand2FWS2RWD.rearRightWheelSetPoint);

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
