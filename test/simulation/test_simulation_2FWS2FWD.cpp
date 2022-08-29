// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_core_mobile_base/simulation/SimulationControl2FWS2FWD.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"

romea::HardwareCommand2FWS2FWD toHardwareCommand2FWS2FWD(const double & frontWheelRadius,
                                                         const romea::OdometryFrame2FWS2FWD & odometryFrame)
{
  return {odometryFrame.frontLeftWheelSteeringAngle,
        odometryFrame.frontRightWheelSteeringAngle,
        odometryFrame.frontLeftWheelLinearSpeed/frontWheelRadius,
        odometryFrame.frontRightWheelLinearSpeed/frontWheelRadius};
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

class TestSimulation2FWS2FWD : public ::testing::Test
{

public :


  TestSimulation2FWS2FWD(){}

  virtual void SetUp()override
  {
    frontWheelRadius = 0.4;
    rearWheelRadius = 0.6;
    parameters.frontWheelBase= 0.8;
    parameters.rearWheelBase= 0.6;
    parameters.frontWheelTrack=1.2;
    parameters.rearWheelTrack=1.4;
    parameters.frontHubCarrierOffset=0.1;
    parameters.rearHubCarrierOffset=0.15;

    command.longitudinalSpeed= 1.;
    command.steeringAngle = 0.4;

    romea::OdometryFrame2FWS2FWD odometryCommand;
    romea::forwardKinematic(parameters,command,odometryCommand);
    hardwareCommand2FWS2FWD = toHardwareCommand2FWS2FWD(frontWheelRadius,
                                                        odometryCommand);

    simulationCommand2FWS2FWD = toSimulationCommand2FWS2FWD(parameters.frontWheelBase+
                                                            parameters.rearWheelBase,
                                                            parameters.frontWheelTrack,
                                                            parameters.rearWheelTrack,
                                                            frontWheelRadius,
                                                            rearWheelRadius,
                                                            parameters.frontHubCarrierOffset,
                                                            parameters.rearHubCarrierOffset,
                                                            hardwareCommand2FWS2FWD);

  }


  double frontWheelRadius;
  double rearWheelRadius;
  romea::TwoWheelSteeringKinematic::Parameters parameters;

  romea::OneAxleSteeringCommand command;
  romea::HardwareCommand2FWS2FWD hardwareCommand2FWS2FWD;
  romea::SimulationCommand2FWS2FWD simulationCommand2FWS2FWD;

};


TEST_F(TestSimulation2FWS2FWD,toSimulation)
{

  romea::OdometryFrame2FWS4WD odometryCommand2FWS4WD;
  romea::forwardKinematic(parameters,command,odometryCommand2FWS4WD);
  auto hardwareCommand2FWS4WD = toHardwareCommand2FWS4WD(frontWheelRadius,
                                                         rearWheelRadius,
                                                         odometryCommand2FWS4WD);

  EXPECT_DOUBLE_EQ(simulationCommand2FWS2FWD.frontLeftWheelSetPoint,
                   hardwareCommand2FWS4WD.frontLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2FWD.frontRightWheelSetPoint,
                   hardwareCommand2FWS4WD.frontRightWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2FWD.frontLeftWheelSteeringAngle,
                   hardwareCommand2FWS4WD.frontLeftWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2FWD.frontRightWheelSteeringAngle,
                   hardwareCommand2FWS2FWD.frontRightWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2FWD.frontLeftWheelSetPoint,
                   hardwareCommand2FWS4WD.frontLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2FWD.frontRightWheelSetPoint,
                   hardwareCommand2FWS4WD.frontRightWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2FWD.rearLeftWheelSetPoint,
                   hardwareCommand2FWS4WD.rearLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand2FWS2FWD.rearRightWheelSetPoint,
                   hardwareCommand2FWS4WD.rearRightWheelSetPoint);
}


TEST_F(TestSimulation2FWS2FWD,toHardware)
{

  romea::SimulationState2FWSxxx simulationState;
  simulationState.frontLeftWheelSteeringAngle = simulationCommand2FWS2FWD.frontLeftWheelSteeringAngle;
  simulationState.frontRightWheelSteeringAngle = simulationCommand2FWS2FWD.frontRightWheelSteeringAngle;
  simulationState.frontLeftWheelSpinMotion.velocity = simulationCommand2FWS2FWD.frontLeftWheelSetPoint;
  simulationState.frontRightWheelSpinMotion.velocity = simulationCommand2FWS2FWD.frontRightWheelSetPoint;
  simulationState.rearLeftWheelSpinMotion.velocity = simulationCommand2FWS2FWD.rearLeftWheelSetPoint;
  simulationState.rearRightWheelSpinMotion.velocity = simulationCommand2FWS2FWD.rearRightWheelSetPoint;

  auto hardwareState2FWS2FWD =romea::toHardwareState2FWS2FWD(simulationState);

  EXPECT_DOUBLE_EQ(hardwareState2FWS2FWD.frontLeftWheelSteeringAngle,
                   hardwareCommand2FWS2FWD.frontLeftWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(hardwareState2FWS2FWD.frontRightWheelSteeringAngle,
                   hardwareCommand2FWS2FWD.frontRightWheelSteeringAngle);

  EXPECT_DOUBLE_EQ(hardwareState2FWS2FWD.frontLeftWheelSpinMotion.velocity,
                   hardwareCommand2FWS2FWD.frontLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(hardwareState2FWS2FWD.frontRightWheelSpinMotion.velocity,
                   hardwareCommand2FWS2FWD.frontRightWheelSetPoint);

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
