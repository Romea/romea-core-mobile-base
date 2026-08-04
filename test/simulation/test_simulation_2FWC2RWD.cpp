// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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
#include "romea_core_mobile_base/simulation/SimulationControl2FWC2RWD.hpp"

class TestSimulation2FWC2RWD : public ::testing::Test
{
public:
  void SetUp() override
  {
    wheelbase = 1.2;
    frontTrack = 1.4;
    frontWheelXOffset = 0.1;
    frontWheelRadius = 0.5;
    rearWheelRadius = 0.3;

    frontLeftWheelSwivelingAngle = -0.4;
    frontRightWheelSwivelingAngle = -0.3;
    hardwareCommand.rearLeftWheelSpinningSetPoint = 2.0;
    hardwareCommand.rearRightWheelSpinningSetPoint = 3.0;

    simulationCommand = romea::core::toSimulationCommand2FWC2RWD(
      hardwareCommand);
  }

  double wheelbase;
  double frontTrack;
  double frontWheelXOffset;
  double frontWheelRadius;
  double rearWheelRadius;
  romea::core::SteeringAngleState frontLeftWheelSwivelingAngle;
  romea::core::SteeringAngleState frontRightWheelSwivelingAngle;
  romea::core::HardwareCommand2FWC2RWD hardwareCommand;
  romea::core::SimulationCommand2FWC2RWD simulationCommand;
};

TEST_F(TestSimulation2FWC2RWD, toSimulationCommand)
{
  EXPECT_DOUBLE_EQ(
    simulationCommand.rearLeftWheelSpinningSetPoint,
    hardwareCommand.rearLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    simulationCommand.rearRightWheelSpinningSetPoint,
    hardwareCommand.rearRightWheelSpinningSetPoint);
}

TEST_F(TestSimulation2FWC2RWD, toSimulationState)
{
  romea::core::HardwareState2FWC2RWD hardwareState;
  hardwareState.frontLeftWheelSwivelingAngle = frontLeftWheelSwivelingAngle;
  hardwareState.frontRightWheelSwivelingAngle = frontRightWheelSwivelingAngle;
  hardwareState.rearLeftWheelSpinningMotion.position = 1.0;
  hardwareState.rearLeftWheelSpinningMotion.velocity =
    hardwareCommand.rearLeftWheelSpinningSetPoint;
  hardwareState.rearLeftWheelSpinningMotion.torque = 4.0;
  hardwareState.rearRightWheelSpinningMotion.position = 2.0;
  hardwareState.rearRightWheelSpinningMotion.velocity =
    hardwareCommand.rearRightWheelSpinningSetPoint;
  hardwareState.rearRightWheelSpinningMotion.torque = 5.0;

  auto simulationState = romea::core::toSimulationState2FWC2RWD(
    wheelbase,
    frontTrack,
    frontWheelXOffset,
    frontWheelRadius,
    rearWheelRadius,
    hardwareState);

  EXPECT_DOUBLE_EQ(
    simulationState.frontLeftWheelSwivelingAngle,
    hardwareState.frontLeftWheelSwivelingAngle);
  EXPECT_DOUBLE_EQ(
    simulationState.frontRightWheelSwivelingAngle,
    hardwareState.frontRightWheelSwivelingAngle);
  EXPECT_DOUBLE_EQ(
    simulationState.rearLeftWheelSpinningMotion.position,
    hardwareState.rearLeftWheelSpinningMotion.position);
  EXPECT_DOUBLE_EQ(
    simulationState.rearLeftWheelSpinningMotion.velocity,
    hardwareState.rearLeftWheelSpinningMotion.velocity);
  EXPECT_DOUBLE_EQ(
    simulationState.rearLeftWheelSpinningMotion.torque,
    hardwareState.rearLeftWheelSpinningMotion.torque);
  EXPECT_DOUBLE_EQ(
    simulationState.rearRightWheelSpinningMotion.position,
    hardwareState.rearRightWheelSpinningMotion.position);
  EXPECT_DOUBLE_EQ(
    simulationState.rearRightWheelSpinningMotion.velocity,
    hardwareState.rearRightWheelSpinningMotion.velocity);
  EXPECT_DOUBLE_EQ(
    simulationState.rearRightWheelSpinningMotion.torque,
    hardwareState.rearRightWheelSpinningMotion.torque);
}

TEST_F(TestSimulation2FWC2RWD, toHardware)
{
  romea::core::SimulationState2FWC2RWD simulationState;
  simulationState.frontLeftWheelSwivelingAngle = frontLeftWheelSwivelingAngle;
  simulationState.frontRightWheelSwivelingAngle = frontRightWheelSwivelingAngle;
  simulationState.rearLeftWheelSpinningMotion.velocity =
    simulationCommand.rearLeftWheelSpinningSetPoint;
  simulationState.rearRightWheelSpinningMotion.velocity =
    simulationCommand.rearRightWheelSpinningSetPoint;

  auto hardwareState = romea::core::toHardwareState2FWC2RWD(simulationState);

  EXPECT_DOUBLE_EQ(
    hardwareState.frontLeftWheelSwivelingAngle,
    frontLeftWheelSwivelingAngle);
  EXPECT_DOUBLE_EQ(
    hardwareState.frontRightWheelSwivelingAngle,
    frontRightWheelSwivelingAngle);
  EXPECT_DOUBLE_EQ(
    hardwareState.rearLeftWheelSpinningMotion.velocity,
    hardwareCommand.rearLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    hardwareState.rearRightWheelSpinningMotion.velocity,
    hardwareCommand.rearRightWheelSpinningSetPoint);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
