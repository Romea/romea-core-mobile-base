// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_core_mobile_base/simulation/SimulationControl1FAS2FWD.hpp"
#include "romea_core_mobile_base/simulation/SimulationControl2FWS4WD.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"

romea::HardwareCommand1FAS2FWD toHardwareCommand1FAS2FWD(const double & frontWheelRadius,
                                                         const romea::OdometryFrame1FAS2FWD & odometryFrame)
{
  return {odometryFrame.frontAxleSteeringAngle,
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

class TestSimulation1AS2FWD : public ::testing::Test
{

public :


  TestSimulation1AS2FWD(){}

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
    command.steeringAngle = 0.3;

    romea::OdometryFrame1FAS2FWD odometryCommand;
    romea::forwardKinematic(parameters,command,odometryCommand);
    hardwareCommand1FAS2FWD = toHardwareCommand1FAS2FWD(frontWheelRadius,
                                                        odometryCommand);

    simulationCommand1FA2FWD = toSimulationCommand1FAS2FWD(parameters.frontWheelBase+
                                                           parameters.rearWheelBase,
                                                           parameters.frontWheelTrack,
                                                           parameters.rearWheelTrack,
                                                           frontWheelRadius,
                                                           rearWheelRadius,
                                                           parameters.frontHubCarrierOffset,
                                                           parameters.rearHubCarrierOffset,
                                                           hardwareCommand1FAS2FWD);

  }


  double frontWheelRadius;
  double rearWheelRadius;
  romea::OneAxleSteeringKinematic::Parameters parameters;

  romea::OneAxleSteeringCommand command;
  romea::HardwareCommand1FAS2FWD hardwareCommand1FAS2FWD;
  romea::SimulationCommand1FAS2FWD simulationCommand1FA2FWD;

};


TEST_F(TestSimulation1AS2FWD,toSimulation)
{
  romea::TwoWheelSteeringKinematic::Parameters parameters2;
  parameters2.frontWheelBase=parameters.frontWheelBase;
  parameters2.rearWheelBase=parameters.rearWheelBase;
  parameters2.frontWheelTrack=parameters.frontWheelTrack;
  parameters2.rearWheelTrack=parameters.rearWheelTrack;
  parameters2.frontHubCarrierOffset=parameters.frontHubCarrierOffset;
  parameters2.rearHubCarrierOffset=parameters.rearHubCarrierOffset;

  romea::OdometryFrame2FWS4WD odometryCommand2FWS4WD;
  romea::forwardKinematic(parameters2,command,odometryCommand2FWS4WD);
  auto hardwareCommand2FWS4WD = toHardwareCommand2FWS4WD(frontWheelRadius,
                                                         rearWheelRadius,
                                                         odometryCommand2FWS4WD);

  EXPECT_DOUBLE_EQ(simulationCommand1FA2FWD.frontAxleSteeringAngle,
                   command.steeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand1FA2FWD.frontLeftWheelSteeringAngle,
                   hardwareCommand2FWS4WD.frontLeftWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand1FA2FWD.frontRightWheelSteeringAngle,
                   hardwareCommand2FWS4WD.frontRightWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand1FA2FWD.frontLeftWheelSetPoint,
                   hardwareCommand2FWS4WD.frontLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand1FA2FWD.frontRightWheelSetPoint,
                   hardwareCommand2FWS4WD.frontRightWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand1FA2FWD.rearLeftWheelSetPoint,
                   hardwareCommand2FWS4WD.rearLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand1FA2FWD.rearRightWheelSetPoint,
                   hardwareCommand2FWS4WD.rearRightWheelSetPoint);
}


TEST_F(TestSimulation1AS2FWD,toHardware)
{

  romea::SimulationState1FASxxx simulationState;
  simulationState.frontAxleSteeringAngle = command.steeringAngle;
  simulationState.frontLeftWheelSteeringAngle = simulationCommand1FA2FWD.frontLeftWheelSteeringAngle;
  simulationState.frontRightWheelSteeringAngle = simulationCommand1FA2FWD.frontRightWheelSteeringAngle;
  simulationState.frontLeftWheelSpinMotion.velocity = simulationCommand1FA2FWD.frontLeftWheelSetPoint;
  simulationState.frontRightWheelSpinMotion.velocity = simulationCommand1FA2FWD.frontRightWheelSetPoint;
  simulationState.rearLeftWheelSpinMotion.velocity = simulationCommand1FA2FWD.rearLeftWheelSetPoint;
  simulationState.rearRightWheelSpinMotion.velocity = simulationCommand1FA2FWD.rearRightWheelSetPoint;

  auto hardwareState1FAS2FWD =romea::toHardwareState1FAS2FWD(parameters.rearWheelBase+
                                                             parameters.frontWheelBase,
                                                             parameters.frontWheelTrack,
                                                             simulationState);

  EXPECT_DOUBLE_EQ(hardwareState1FAS2FWD.frontAxleSteeringAngle,
                   hardwareCommand1FAS2FWD.frontAxleSteeringAngle);
  EXPECT_DOUBLE_EQ(hardwareState1FAS2FWD.frontLeftWheelSpinMotion.velocity,
                   hardwareCommand1FAS2FWD.frontLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(hardwareState1FAS2FWD.frontRightWheelSpinMotion.velocity,
                   hardwareCommand1FAS2FWD.frontRightWheelSetPoint);

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
