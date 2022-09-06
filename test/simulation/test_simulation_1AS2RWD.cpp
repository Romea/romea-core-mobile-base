// gtest
#include <gtest/gtest.h>

//romea
#include "romea_core_mobile_base/simulation/SimulationControl1FAS2RWD.hpp"
#include "romea_core_mobile_base/simulation/SimulationControl2FWS4WD.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"

romea::HardwareCommand1FAS2RWD toHardwareCommand1FAS2RWD(const double & rearWheelRadius,
                                                         const romea::OdometryFrame1FAS2RWD & odometryFrame)
{
  return {odometryFrame.frontAxleSteeringAngle,
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

class TestSimulation1AS2RWD : public ::testing::Test
{

public :


  TestSimulation1AS2RWD(){}

  virtual void SetUp()override
  {
    frontWheelRadius = 0.3;
    rearWheelRadius = 0.7;
    parameters.frontWheelBase= 0.9;
    parameters.rearWheelBase= 0.4;
    parameters.frontWheelTrack=0.8;
    parameters.rearWheelTrack=1.1;
    parameters.frontHubCarrierOffset=0.08;
    parameters.rearHubCarrierOffset=0.12;

    command.longitudinalSpeed= -1.;
    command.steeringAngle = -0.5;

    romea::OdometryFrame1FAS2RWD odometryCommand;
    romea::forwardKinematic(parameters,command,odometryCommand);
    hardwareCommand1FAS2RWD = toHardwareCommand1FAS2RWD(rearWheelRadius,
                                                        odometryCommand);

    simulationCommand1FAS2RWD = toSimulationCommand1FAS2RWD(parameters.frontWheelBase+
                                                            parameters.rearWheelBase,
                                                            parameters.frontWheelTrack,
                                                            parameters.frontHubCarrierOffset,
                                                            frontWheelRadius,
                                                            rearWheelRadius,
                                                            hardwareCommand1FAS2RWD);

  }


  double frontWheelRadius;
  double rearWheelRadius;
  romea::OneAxleSteeringKinematic::Parameters parameters;

  romea::OneAxleSteeringCommand command;
  romea::HardwareCommand1FAS2RWD hardwareCommand1FAS2RWD;
  romea::SimulationCommand1FAS2RWD simulationCommand1FAS2RWD;

};


TEST_F(TestSimulation1AS2RWD,toSimulation)
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

  EXPECT_DOUBLE_EQ(simulationCommand1FAS2RWD.frontAxleSteeringAngle,
                   command.steeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand1FAS2RWD.frontLeftWheelSteeringAngle,
                   hardwareCommand2FWS4WD.frontLeftWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand1FAS2RWD.frontRightWheelSteeringAngle,
                   hardwareCommand2FWS4WD.frontRightWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand1FAS2RWD.frontLeftWheelSetPoint,
                   hardwareCommand2FWS4WD.frontLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand1FAS2RWD.frontRightWheelSetPoint,
                   hardwareCommand2FWS4WD.frontRightWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand1FAS2RWD.rearLeftWheelSetPoint,
                   hardwareCommand2FWS4WD.rearLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand1FAS2RWD.rearRightWheelSetPoint,
                   hardwareCommand2FWS4WD.rearRightWheelSetPoint);
}


TEST_F(TestSimulation1AS2RWD,toHardware)
{

  romea::SimulationState1FASxxx simulationState;
  simulationState.frontAxleSteeringAngle = command.steeringAngle;
  simulationState.frontLeftWheelSteeringAngle = simulationCommand1FAS2RWD.frontLeftWheelSteeringAngle;
  simulationState.frontRightWheelSteeringAngle = simulationCommand1FAS2RWD.frontRightWheelSteeringAngle;
  simulationState.frontLeftWheelSpinMotion.velocity = simulationCommand1FAS2RWD.frontLeftWheelSetPoint;
  simulationState.frontRightWheelSpinMotion.velocity = simulationCommand1FAS2RWD.frontRightWheelSetPoint;
  simulationState.rearLeftWheelSpinMotion.velocity = simulationCommand1FAS2RWD.rearLeftWheelSetPoint;
  simulationState.rearRightWheelSpinMotion.velocity = simulationCommand1FAS2RWD.rearRightWheelSetPoint;

  auto hardwareState1FAS2RWD =romea::toHardwareState1FAS2RWD(parameters.rearWheelBase+
                                                             parameters.frontWheelBase,
                                                             parameters.frontWheelTrack,
                                                             simulationState);

  EXPECT_DOUBLE_EQ(hardwareState1FAS2RWD.frontAxleSteeringAngle,
                   hardwareCommand1FAS2RWD.frontAxleSteeringAngle);
  EXPECT_DOUBLE_EQ(hardwareState1FAS2RWD.rearLeftWheelSpinMotion.velocity,
                   hardwareCommand1FAS2RWD.rearLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(hardwareState1FAS2RWD.rearRightWheelSpinMotion.velocity,
                   hardwareCommand1FAS2RWD.rearRightWheelSetPoint);

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
