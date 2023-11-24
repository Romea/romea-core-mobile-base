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
#include "romea_core_mobile_base/simulation/SimulationControl2TTD.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"

romea::core::HardwareCommand2TD toHardwareCommand2TD(
  const double & sprocketWheelRadius,
  const double & trackThickness,
  const romea::core::OdometryFrame2TD & odometryFrame)
{
  return {odometryFrame.leftTrackLinearSpeed / (sprocketWheelRadius + trackThickness),
    odometryFrame.rightTrackLinearSpeed / (sprocketWheelRadius + trackThickness)};
}

class TestSimulation2TTD : public ::testing::Test
{
public:
  TestSimulation2TTD() {}

  void SetUp()override
  {
    sprocketWheelRadius = 0.8;
    idlerWheelRadius = 0.3;
    rollerWheelRadius = 0.2;
    trackThickness = 0.1;
    parameters.wheelTrack = 1.5;

    command.longitudinalSpeed = 1.;
    command.angularSpeed = 0.6;

    romea::core::OdometryFrame2TD odometryCommand;
    romea::core::forwardKinematic(parameters, command, odometryCommand);
    hardwareCommand2TD = toHardwareCommand2TD(
      sprocketWheelRadius,
      trackThickness,
      odometryCommand);

    std::cout << "hardwareCommand2TD " << odometryCommand << std::endl;

    simulationCommand2TTD = toSimulationCommand2TTD(
      sprocketWheelRadius,
      rollerWheelRadius,
      idlerWheelRadius,
      trackThickness,
      hardwareCommand2TD);

    std::cout << "simulationCommand2TTD " << simulationCommand2TTD << std::endl;
  }

  double sprocketWheelRadius;
  double rollerWheelRadius;
  double idlerWheelRadius;
  double trackThickness;
  romea::core::SkidSteeringKinematic::Parameters parameters;

  romea::core::SkidSteeringCommand command;
  romea::core::HardwareCommand2TD hardwareCommand2TD;
  romea::core::SimulationCommand2TTD simulationCommand2TTD;
};


TEST_F(TestSimulation2TTD, toSimulation)
{
  EXPECT_DOUBLE_EQ(
    simulationCommand2TTD.leftSprocketWheelSpinningSetPoint,
    hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_DOUBLE_EQ(
    simulationCommand2TTD.rightSprocketWheelSpinningSetPoint,
    hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2TTD.leftIdlerWheelSpinningSetPoint,
    hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2TTD.rightIdlerWheelSpinningSetPoint,
    hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2TTD.frontLeftRollerWheelSpinningSetPoint,
    hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2TTD.frontRightRollerWheelSpinningSetPoint,
    hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2TTD.rearLeftRollerWheelSpinningSetPoint,
    hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2TTD.rearRightRollerWheelSpinningSetPoint,
    hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);
}


TEST_F(TestSimulation2TTD, toHardware)
{
  romea::core::SimulationState2TTD simulationState;
  simulationState.leftSprocketWheelSpinningMotion.velocity =
    simulationCommand2TTD.leftSprocketWheelSpinningSetPoint * 2;
  simulationState.rightSprocketWheelSpinningMotion.velocity =
    simulationCommand2TTD.rightSprocketWheelSpinningSetPoint * 2;
  simulationState.leftIdlerWheelSpinningMotion.velocity =
    simulationCommand2TTD.leftIdlerWheelSpinningSetPoint * 2;
  simulationState.rightIdlerWheelSpinningMotion.velocity =
    simulationCommand2TTD.rightIdlerWheelSpinningSetPoint * 2;
  simulationState.frontLeftRollerWheelSpinningMotion.velocity =
    simulationCommand2TTD.frontLeftRollerWheelSpinningSetPoint;
  simulationState.rearLeftRollerWheelSpinningMotion.velocity =
    simulationCommand2TTD.rearLeftRollerWheelSpinningSetPoint + 0.1;
  simulationState.frontRightRollerWheelSpinningMotion.velocity =
    simulationCommand2TTD.frontRightRollerWheelSpinningSetPoint + 0.1;
  simulationState.rearRightRollerWheelSpinningMotion.velocity =
    simulationCommand2TTD.rearRightRollerWheelSpinningSetPoint;


  auto hardwareState2TD = romea::core::toHardwareState2TTD(
    sprocketWheelRadius,
    rollerWheelRadius,
    trackThickness,
    simulationState);

  EXPECT_DOUBLE_EQ(
    hardwareState2TD.leftSprocketWheelSpinningMotion.velocity,
    hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);
  EXPECT_DOUBLE_EQ(
    hardwareState2TD.rightSprocketWheelSpinningMotion.velocity,
    hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
