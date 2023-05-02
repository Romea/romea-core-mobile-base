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
#include "romea_core_mobile_base/simulation/SimulationControl2TD.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"

romea::HardwareCommand2TD toHardwareCommand2TD(
  const double & sprocketWheelRadius,
  const double & trackWidth,
  const romea::OdometryFrame2TD & odometryFrame)
{
  return {odometryFrame.leftTrackLinearSpeed / (sprocketWheelRadius + trackWidth),
    odometryFrame.rightTrackLinearSpeed / (sprocketWheelRadius + trackWidth)};
}


class TestSimulation2TD : public ::testing::Test
{
public:
  TestSimulation2TD() {}

  void SetUp()override
  {
    sprocketWheelRadius = 0.8;
    idlerWheelRadius = 0.3;
    trackThickness = 0.1;
    parameters.wheelTrack = 1.5;

    command.longitudinalSpeed = 1.;
    command.angularSpeed = 0.6;

    romea::OdometryFrame2TD odometryCommand;
    romea::forwardKinematic(parameters, command, odometryCommand);
    hardwareCommand2TD = toHardwareCommand2TD(
      sprocketWheelRadius,
      trackThickness,
      odometryCommand);

    std::cout << "hardwareCommand2TD " << hardwareCommand2TD << std::endl;

    simulationCommand2TD = toSimulationCommand2TD(
      sprocketWheelRadius,
      idlerWheelRadius,
      trackThickness,
      hardwareCommand2TD);

    std::cout << "simulationCommand2TD " << simulationCommand2TD << std::endl;

  }

  double sprocketWheelRadius;
  double idlerWheelRadius;
  double trackThickness;
  romea::SkidSteeringKinematic::Parameters parameters;

  romea::SkidSteeringCommand command;
  romea::HardwareCommand2TD hardwareCommand2TD;
  romea::SimulationCommand2TD simulationCommand2TD;
};


TEST_F(TestSimulation2TD, toSimulation)
{
  EXPECT_DOUBLE_EQ(
    simulationCommand2TD.leftSprocketWheelSpinningSetPoint,
    hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_DOUBLE_EQ(
    simulationCommand2TD.rightSprocketWheelSpinningSetPoint,
    hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2TD.leftIdlerWheelSpinningSetPoint,
    hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2TD.rightIdlerWheelSpinningSetPoint,
    hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);
}


TEST_F(TestSimulation2TD, toHardware)
{
  romea::SimulationState2TD simulationState;
  simulationState.leftSprocketWheelSpinningMotion.velocity =
    simulationCommand2TD.leftSprocketWheelSpinningSetPoint;
  simulationState.leftIdlerWheelSpinningMotion.velocity =
    simulationCommand2TD.leftIdlerWheelSpinningSetPoint + 0.1;
  simulationState.rightSprocketWheelSpinningMotion.velocity =
    simulationCommand2TD.rightSprocketWheelSpinningSetPoint + 0.1;
  simulationState.rightIdlerWheelSpinningMotion.velocity =
    simulationCommand2TD.rightIdlerWheelSpinningSetPoint;

  std::cout << " simulation command " << std::endl;
  std::cout << simulationCommand2TD.leftSprocketWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2TD.rightSprocketWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2TD.leftIdlerWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2TD.rightIdlerWheelSpinningSetPoint << std::endl;

  auto hardwareState2TD = romea::toHardwareState2TD(
    sprocketWheelRadius,
    idlerWheelRadius,
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
