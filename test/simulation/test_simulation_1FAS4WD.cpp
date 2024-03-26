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
#include "romea_core_mobile_base/simulation/SimulationControl1FAS4WD.hpp"
#include "romea_core_mobile_base/simulation/SimulationControl2FWS4WD.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"

romea::core::HardwareCommand1FAS4WD toHardwareCommand1FAS4WD(
  const double & frontWheelRadius,
  const double & rearWheelRadius,
  const romea::core::OdometryFrame1FAS4WD & odometryFrame)
{
  return {odometryFrame.frontAxleSteeringAngle,
    odometryFrame.frontLeftWheelLinearSpeed / frontWheelRadius,
    odometryFrame.frontRightWheelLinearSpeed / frontWheelRadius,
    odometryFrame.rearLeftWheelLinearSpeed / rearWheelRadius,
    odometryFrame.rearRightWheelLinearSpeed / rearWheelRadius};
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

class TestSimulation1FAS4WD : public ::testing::Test
{
public:
  TestSimulation1FAS4WD() {}

  void SetUp()override
  {
    frontWheelRadius = 0.3;
    rearWheelRadius = 0.7;
    parameters.frontWheelBase = 0.9;
    parameters.rearWheelBase = 0.4;
    parameters.frontWheelTrack = 0.8;
    parameters.rearWheelTrack = 1.1;
    parameters.frontHubCarrierOffset = 0.08;
    parameters.rearHubCarrierOffset = 0.12;

    command.longitudinalSpeed = -1.;
    command.steeringAngle = -0.5;

    romea::core::OdometryFrame1FAS4WD odometryCommand;
    romea::core::forwardKinematic(parameters, command, odometryCommand);
    hardwareCommand1FAS4WD = toHardwareCommand1FAS4WD(
      frontWheelRadius,
      rearWheelRadius,
      odometryCommand);

    simulationCommand1FAS4WD = toSimulationCommand1FAS4WD(
      parameters.frontWheelBase +
      parameters.rearWheelBase,
      parameters.frontWheelTrack,
      hardwareCommand1FAS4WD);

  }


  double frontWheelRadius;
  double rearWheelRadius;
  romea::core::OneAxleSteeringKinematic::Parameters parameters;

  romea::core::OneAxleSteeringCommand command;
  romea::core::HardwareCommand1FAS4WD hardwareCommand1FAS4WD;
  romea::core::SimulationCommand1FAS4WD simulationCommand1FAS4WD;

};


TEST_F(TestSimulation1FAS4WD, toSimulation)
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
    simulationCommand1FAS4WD.frontAxleSteeringAngle,
    command.steeringAngle);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS4WD.frontLeftWheelSteeringAngle,
    hardwareCommand2FWS4WD.frontLeftWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS4WD.frontRightWheelSteeringAngle,
    hardwareCommand2FWS4WD.frontRightWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS4WD.frontLeftWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.frontLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS4WD.frontRightWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.frontRightWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS4WD.rearLeftWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.rearLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS4WD.rearRightWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.rearRightWheelSpinningSetPoint);
}


TEST_F(TestSimulation1FAS4WD, toHardware)
{
  romea::core::SimulationState1FASxxx simulationState;
  simulationState.frontAxleSteeringAngle =
    command.steeringAngle;
  simulationState.frontLeftWheelSteeringAngle =
    simulationCommand1FAS4WD.frontLeftWheelSteeringAngle;
  simulationState.frontRightWheelSteeringAngle =
    simulationCommand1FAS4WD.frontRightWheelSteeringAngle;
  simulationState.frontLeftWheelSpinningMotion.velocity =
    simulationCommand1FAS4WD.frontLeftWheelSpinningSetPoint;
  simulationState.frontRightWheelSpinningMotion.velocity =
    simulationCommand1FAS4WD.frontRightWheelSpinningSetPoint;
  simulationState.rearLeftWheelSpinningMotion.velocity =
    simulationCommand1FAS4WD.rearLeftWheelSpinningSetPoint;
  simulationState.rearRightWheelSpinningMotion.velocity =
    simulationCommand1FAS4WD.rearRightWheelSpinningSetPoint;

  auto hardwareState1FAS4WD = toHardwareState1FAS4WD(
    parameters.rearWheelBase +
    parameters.frontWheelBase,
    parameters.frontWheelTrack,
    simulationState);

  EXPECT_DOUBLE_EQ(
    hardwareState1FAS4WD.frontAxleSteeringAngle,
    hardwareCommand1FAS4WD.frontAxleSteeringAngle);
  EXPECT_DOUBLE_EQ(
    hardwareState1FAS4WD.frontLeftWheelSpinningMotion.velocity,
    hardwareCommand1FAS4WD.frontLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    hardwareState1FAS4WD.frontRightWheelSpinningMotion.velocity,
    hardwareCommand1FAS4WD.frontRightWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    hardwareState1FAS4WD.rearLeftWheelSpinningMotion.velocity,
    hardwareCommand1FAS4WD.rearLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    hardwareState1FAS4WD.rearRightWheelSpinningMotion.velocity,
    hardwareCommand1FAS4WD.rearRightWheelSpinningSetPoint);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
