// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include "romea_core_mobile_base/simulation/SimulationControl1FWS2FWD.hpp"
//#include "romea_core_mobile_base/simulation/SimulationControl2FWS4WD.hpp"
//#include "romea_core_mobile_base/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
//#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"

//romea::HardwareCommand1FAS2FWD toHardwareCommand1FAS2FWD(const double & frontWheelRadius,
//                                                         const romea::OdometryFrame1FAS2FWD & odometryFrame)
//{
//  return {odometryFrame.frontAxleSteeringAngle,
//        odometryFrame.frontLeftWheelLinearSpeed/frontWheelRadius,
//        odometryFrame.frontRightWheelLinearSpeed/frontWheelRadius};
//}

//romea::HardwareCommand2FWS4WD toHardwareCommand2FWS4WD(const double & frontWheelRadius,
//                                                       const double & rearWheelRadius,
//                                                       const romea::OdometryFrame2FWS4WD & odometryFrame)
//{
//  return {odometryFrame.frontLeftWheelSteeringAngle,
//        odometryFrame.frontRightWheelSteeringAngle,
//        odometryFrame.frontLeftWheelLinearSpeed/frontWheelRadius,
//        odometryFrame.frontRightWheelLinearSpeed/frontWheelRadius,
//        odometryFrame.rearLeftWheelLinearSpeed/rearWheelRadius,
//        odometryFrame.rearRightWheelLinearSpeed/rearWheelRadius};
//}

class TestSimulation1FWS2RWD : public ::testing::Test
{

public :


  TestSimulation1FWS2RWD(){}

  virtual void SetUp()override
  {
    frontWheelRadius = 0.4;
    rearWheelRadius = 0.6;
//    parameters.frontWheelBase= 0.8;
//    parameters.rearWheelBase= 0.6;
//    parameters.frontWheelTrack=1.2;
//    parameters.rearWheelTrack=1.4;
//    parameters.frontHubCarrierOffset=0.1;
//    parameters.rearHubCarrierOffset=0.15;

    command.longitudinalSpeed= 1.;
    command.steeringAngle = 0.3;

    romea::OdometryFrame1FAS2FWD odometryCommand;
    romea::forwardKinematic(parameters,command,odometryCommand);
    hardwareCommand = toHardwareCommand1FAS2FWD(frontWheelRadius,
                                                odometryCommand);

    simulationCommand = toSimulationCommand1FAS2FWD(parameters.frontWheelBase+
                                                    parameters.rearWheelBase,
                                                    parameters.frontWheelTrack,
                                                    parameters.rearWheelTrack,
                                                    frontWheelRadius,
                                                    rearWheelRadius,
                                                    parameters.frontHubCarrierOffset,
                                                    parameters.rearHubCarrierOffset,
                                                    hardwareCommand);

  }


  double frontWheelRadius;
  double rearWheelRadius;
  romea::OneAxleSteeringKinematic::Parameters parameters;

  romea::OneAxleSteeringCommand command;
  romea::HardwareCommand1FAS2FWD hardwareCommand;
  romea::SimulationCommand1FAS2FWD simulationCommand;

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

  romea::OdometryFrame2FWS4WD odometryCommand2;
  romea::forwardKinematic(parameters2,command,odometryCommand2);
  auto hardwareCommand2 = toHardwareCommand2FWS4WD(frontWheelRadius,
                                                      rearWheelRadius,
                                                      odometryCommand2);

  EXPECT_DOUBLE_EQ(simulationCommand.frontAxleSteeringAngle,
                   command.steeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand.frontLeftWheelSteeringAngle,
                   hardwareCommand2.frontLeftWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand.frontRightWheelSteeringAngle,
                   hardwareCommand2.frontRightWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(simulationCommand.frontLeftWheelSetPoint,
                   hardwareCommand2.frontLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand.frontRightWheelSetPoint,
                   hardwareCommand2.frontRightWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand.rearLeftWheelSetPoint,
                   hardwareCommand2.rearLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(simulationCommand.rearRightWheelSetPoint,
                   hardwareCommand2.rearRightWheelSetPoint);
}


TEST_F(TestSimulation1AS2FWD,toHardware)
{

  romea::SimulationState1FASxxx simulationState;
  simulationState.frontAxleSteeringAngle = command.steeringAngle;
  simulationState.frontLeftWheelSteeringAngle = simulationCommand.frontLeftWheelSteeringAngle;
  simulationState.frontRightWheelSteeringAngle = simulationCommand.frontRightWheelSteeringAngle;
  simulationState.frontLeftWheelSpinMotion.velocity = simulationCommand.frontLeftWheelSetPoint;
  simulationState.frontRightWheelSpinMotion.velocity = simulationCommand.frontRightWheelSetPoint;
  simulationState.rearLeftWheelSpinMotion.velocity = simulationCommand.rearLeftWheelSetPoint;
  simulationState.rearRightWheelSpinMotion.velocity = simulationCommand.rearRightWheelSetPoint;

  auto hardwareState =romea::toHardwareState1FAS2FWD(parameters.rearWheelBase+
                                                      parameters.frontWheelBase,
                                                      parameters.frontWheelTrack,
                                                      simulationState);

  EXPECT_DOUBLE_EQ(hardwareState.frontAxleSteeringAngle,
                   hardwareCommand.frontAxleSteeringAngle);
  EXPECT_DOUBLE_EQ(hardwareState.frontLeftWheelSpinMotion.velocity,
                   hardwareCommand.frontLeftWheelSetPoint);
  EXPECT_DOUBLE_EQ(hardwareState.frontRightWheelSpinMotion.velocity,
                   hardwareCommand.frontRightWheelSetPoint);

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
