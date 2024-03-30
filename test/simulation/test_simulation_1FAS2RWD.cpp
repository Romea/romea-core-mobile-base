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
#include "romea_core_mobile_base/simulation/SimulationControl1FAS2RWD.hpp"
#include "romea_core_mobile_base/simulation/SimulationControl2FWS4WD.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"

romea::core::HardwareCommand1FAS2RWD toHardwareCommand1FAS2RWD(
  const double & rearWheelRadius,
  const romea::core::OdometryFrame1FAS2RWD & odometryFrame)
{
  return {odometryFrame.frontAxleSteeringAngle,
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

class TestSimulation1FAS2RWD : public ::testing::Test
{
public:
  TestSimulation1FAS2RWD() {}

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

    romea::core::OdometryFrame1FAS2RWD odometryCommand;
    romea::core::forwardKinematic(parameters, command, odometryCommand);
    hardwareCommand1FAS2RWD = toHardwareCommand1FAS2RWD(
      rearWheelRadius,
      odometryCommand);

    std::cout << hardwareCommand1FAS2RWD << std::endl;

    simulationCommand1FAS2RWD = toSimulationCommand1FAS2RWD(
      parameters.frontWheelBase +
      parameters.rearWheelBase,
      parameters.frontWheelTrack,
      parameters.frontHubCarrierOffset,
      frontWheelRadius,
      rearWheelRadius,
      hardwareCommand1FAS2RWD);
  }


  double frontWheelRadius;
  double rearWheelRadius;
  romea::core::OneAxleSteeringKinematic::Parameters parameters;

  romea::core::OneAxleSteeringCommand command;
  romea::core::HardwareCommand1FAS2RWD hardwareCommand1FAS2RWD;
  romea::core::SimulationCommand1FAS2RWD simulationCommand1FAS2RWD;
};


TEST_F(TestSimulation1FAS2RWD, toSimulation)
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
    simulationCommand1FAS2RWD.frontAxleSteeringAngle,
    command.steeringAngle);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS2RWD.frontLeftWheelSteeringAngle,
    hardwareCommand2FWS4WD.frontLeftWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS2RWD.frontRightWheelSteeringAngle,
    hardwareCommand2FWS4WD.frontRightWheelSteeringAngle);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS2RWD.frontLeftWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.frontLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS2RWD.frontRightWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.frontRightWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS2RWD.rearLeftWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.rearLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    simulationCommand1FAS2RWD.rearRightWheelSpinningSetPoint,
    hardwareCommand2FWS4WD.rearRightWheelSpinningSetPoint);
}


TEST_F(TestSimulation1FAS2RWD, toHardware)
{
  romea::core::SimulationState1FASxxx simulationState;
  simulationState.frontAxleSteeringAngle =
    command.steeringAngle;
  simulationState.frontLeftWheelSteeringAngle =
    simulationCommand1FAS2RWD.frontLeftWheelSteeringAngle;
  simulationState.frontRightWheelSteeringAngle =
    simulationCommand1FAS2RWD.frontRightWheelSteeringAngle;
  simulationState.frontLeftWheelSpinningMotion.velocity =
    simulationCommand1FAS2RWD.frontLeftWheelSpinningSetPoint;
  simulationState.frontRightWheelSpinningMotion.velocity =
    simulationCommand1FAS2RWD.frontRightWheelSpinningSetPoint;
  simulationState.rearLeftWheelSpinningMotion.velocity =
    simulationCommand1FAS2RWD.rearLeftWheelSpinningSetPoint;
  simulationState.rearRightWheelSpinningMotion.velocity =
    simulationCommand1FAS2RWD.rearRightWheelSpinningSetPoint;

  auto hardwareState1FAS2RWD = romea::core::toHardwareState1FAS2RWD(
    parameters.rearWheelBase +
    parameters.frontWheelBase,
    parameters.frontWheelTrack,
    simulationState);

  EXPECT_DOUBLE_EQ(
    hardwareState1FAS2RWD.frontAxleSteeringAngle,
    hardwareCommand1FAS2RWD.frontAxleSteeringAngle);
  EXPECT_DOUBLE_EQ(
    hardwareState1FAS2RWD.rearLeftWheelSpinningMotion.velocity,
    hardwareCommand1FAS2RWD.rearLeftWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    hardwareState1FAS2RWD.rearRightWheelSpinningMotion.velocity,
    hardwareCommand1FAS2RWD.rearRightWheelSpinningSetPoint);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
