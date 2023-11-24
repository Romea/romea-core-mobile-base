// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// gtest
#include <gtest/gtest.h>

// romea
#include "romea_core_mobile_base/simulation/SimulationControl1FAS2FWD.hpp"
#include "romea_core_mobile_base/simulation/SimulationControl2FWS4WD.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"

romea::core::HardwareCommand1FAS2FWD toHardwareCommand1FAS2FWD(
  const double & frontWheelRadius,
  const romea::core::OdometryFrame1FAS2FWD & odometryFrame)
{
  return {odometryFrame.frontAxleSteeringAngle,
    odometryFrame.frontLeftWheelLinearSpeed / frontWheelRadius,
    odometryFrame.frontRightWheelLinearSpeed / frontWheelRadius};
}

romea::core::HardwareCommand2FWS4WD toHardwareCommand2FWS4WD(
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const romea::core::OdometryFrame2FWS4WD & odometryFrame)
{
  return {odometryFrame.frontLeftWheelSteeringAngle,
    odometryFrame.frontRightWheelSteeringAngle,
    odometryFrame.frontLeftWheelLinearSpeed / frontWheelRadius,
    odometryFrame.frontRightWheelLinearSpeed / frontWheelRadius,
    odometryFrame.rearLeftWheelLinearSpeed / rearWheelRadius,
    odometryFrame.rearRightWheelLinearSpeed / rearWheelRadius};
}

class TestSimulation1AS2FWD : public ::testing::Test
{
public:
  TestSimulation1AS2FWD() {}

  void SetUp()override
  {
    frontWheelRadius = 0.4;
    rearWheelRadius = 0.6;
    parameters.frontWheelBase = 0.8;
    parameters.rearWheelBase = 0.6;
    parameters.frontWheelTrack = 1.2;
    parameters.rearWheelTrack = 1.4;
    parameters.frontHubCarrierOffset = 0.1;
    parameters.rearHubCarrierOffset = 0.15;

    command.longitudinalSpeed = 1.;
    command.steeringAngle = 0.3;

    romea::core::OdometryFrame1FAS2FWD odometryCommand;
    romea::core::forwardKinematic(parameters, command, odometryCommand);
    hardwareCommand1FAS2FWD = toHardwareCommand1FAS2FWD(
      frontWheelRadius,
      odometryCommand);

    std::cout << " hardwareCommand1FAS2FWD " << hardwareCommand1FAS2FWD << std::endl;
    simulationCommand1FA2FWD = toSimulationCommand1FAS2FWD(
      parameters.frontWheelBase +
      parameters.rearWheelBase,
      parameters.frontWheelTrack,
      parameters.rearWheelTrack,
      frontWheelRadius,
      rearWheelRadius,
      parameters.frontHubCarrierOffset,
      parameters.rearHubCarrierOffset,
      hardwareCommand1FAS2FWD);

    std::cout << " simulationCommand1FAS2FWD " << simulationCommand1FA2FWD << std::endl;
  }


  double frontWheelRadius;
  double rearWheelRadius;
  romea::core::OneAxleSteeringKinematic::Parameters parameters;

  romea::core::OneAxleSteeringCommand command;
  romea::core::HardwareCommand1FAS2FWD hardwareCommand1FAS2FWD;
  romea::core::SimulationCommand1FAS2FWD simulationCommand1FA2FWD;
};


TEST_F(TestSimulation1AS2FWD, toSimulation)
{
  romea::core::TwoWheelSteeringKinematic::Parameters parameters2;
  parameters2.frontWheelBase = parameters.frontWheelBase;
  parameters2.rearWheelBase = parameters.rearWheelBase;
  parameters2.frontWheelTrack = parameters.frontWheelTrack;
  parameters2.rearWheelTrack = parameters.rearWheelTrack;
  parameters2.frontHubCarrierOffset = parameters.frontHubCarrierOffset;
  parameters2.rearHubCarrierOffset = parameters.rearHubCarrierOffset;

  romea::core::OdometryFrame2FWS4WD odometryCommand2FWS4WD;
  romea::core::forwardKinematic(parameters2, command, odometryCommand2FWS4WD);
  auto hardwareCommand2FWS4WD = toHardwareCommand2FWS4WD(
    frontWheelRadius,
    rearWheelRadius,
    odometryCommand2FWS4WD);

  EXPECT_DOUBLE_EQ(
    simulationCommand1FA2FWD.frontAxleSteeringAngle,
    command.steeringAngle);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FA2FWD.frontLeftWheelSteeringAngle,
    hardwareCommand2FWS4WD.frontLeftWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FA2FWD.frontRightWheelSteeringAngle,
    hardwareCommand2FWS4WD.frontRightWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FA2FWD.frontLeftWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.frontLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FA2FWD.frontRightWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.frontRightWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FA2FWD.rearLeftWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.rearLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FA2FWD.rearRightWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.rearRightWheelSpinningSetPoint);
}


TEST_F(TestSimulation1AS2FWD, toHardware)
{
  romea::core::SimulationState1FASxxx simulationState;
  simulationState.frontAxleSteeringAngle =
    command.steeringAngle;
  simulationState.frontLeftWheelSteeringAngle =
    simulationCommand1FA2FWD.frontLeftWheelSteeringAngle;
  simulationState.frontRightWheelSteeringAngle =
    simulationCommand1FA2FWD.frontRightWheelSteeringAngle;
  simulationState.frontLeftWheelSpinningMotion.velocity =
    simulationCommand1FA2FWD.frontLeftWheelSpinningSetPoint;
  simulationState.frontRightWheelSpinningMotion.velocity =
    simulationCommand1FA2FWD.frontRightWheelSpinningSetPoint;
  simulationState.rearLeftWheelSpinningMotion.velocity =
    simulationCommand1FA2FWD.rearLeftWheelSpinningSetPoint;
  simulationState.rearRightWheelSpinningMotion.velocity =
    simulationCommand1FA2FWD.rearRightWheelSpinningSetPoint;

  auto hardwareState1FAS2FWD = romea::core::toHardwareState1FAS2FWD(
    parameters.rearWheelBase +
    parameters.frontWheelBase,
    parameters.frontWheelTrack,
    simulationState);

  EXPECT_DOUBLE_EQ(
    hardwareState1FAS2FWD.frontAxleSteeringAngle,
    hardwareCommand1FAS2FWD.frontAxleSteeringAngle);
  EXPECT_DOUBLE_EQ(
    hardwareState1FAS2FWD.frontLeftWheelSpinningMotion.velocity,
    hardwareCommand1FAS2FWD.frontLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    hardwareState1FAS2FWD.frontRightWheelSpinningMotion.velocity,
    hardwareCommand1FAS2FWD.frontRightWheelSpinningSetPoint);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
