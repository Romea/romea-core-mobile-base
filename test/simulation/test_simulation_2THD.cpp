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
#include "romea_core_mobile_base/simulation/SimulationControl2THD.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"

romea::core::HardwareCommand2TD toHardwareCommand2TD(
  const double & sprocketWheelRadius,
  const double & trackThickness,
  const romea::core::OdometryFrame2TD & odometryFrame)
{
  return {odometryFrame.leftTrackLinearSpeed / (sprocketWheelRadius + trackThickness),
    odometryFrame.rightTrackLinearSpeed / (sprocketWheelRadius + trackThickness)};
}

class TestSimulation2THD : public ::testing::Test
{
public:
  TestSimulation2THD() {}

  void SetUp()override
  {
    sprocketWheelRadius = 0.8;
    idlerWheelRadius = 0.3;
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

    simulationCommand2THD = toSimulationCommand2THD(
      sprocketWheelRadius,
      idlerWheelRadius,
      trackThickness,
      hardwareCommand2TD);

    std::cout << "simulationCommand2THD " << simulationCommand2THD << std::endl;
  }

  double sprocketWheelRadius;
  double idlerWheelRadius;
  double trackThickness;
  romea::core::SkidSteeringKinematic::Parameters parameters;

  romea::core::SkidSteeringCommand command;
  romea::core::HardwareCommand2TD hardwareCommand2TD;
  romea::core::SimulationCommand2THD simulationCommand2THD;
};


TEST_F(TestSimulation2THD, toSimulation)
{
  std::cout << " simulation command " << std::endl;
  std::cout << simulationCommand2THD.leftSprocketWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2THD.rightSprocketWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2THD.frontLeftIdlerWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2THD.frontRightIdlerWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2THD.rearLeftIdlerWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2THD.rearRightIdlerWheelSpinningSetPoint << std::endl;

  EXPECT_DOUBLE_EQ(
    simulationCommand2THD.leftSprocketWheelSpinningSetPoint,
    hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_DOUBLE_EQ(
    simulationCommand2THD.rightSprocketWheelSpinningSetPoint,
    hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2THD.frontLeftIdlerWheelSpinningSetPoint,
    hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2THD.frontRightIdlerWheelSpinningSetPoint,
    hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2THD.rearLeftIdlerWheelSpinningSetPoint,
    hardwareCommand2TD.leftSprocketWheelSpinningSetPoint);

  EXPECT_GT(
    simulationCommand2THD.rearRightIdlerWheelSpinningSetPoint,
    hardwareCommand2TD.rightSprocketWheelSpinningSetPoint);
}


TEST_F(TestSimulation2THD, toHardware)
{
  romea::core::SimulationState2THD simulationState;
  simulationState.leftSprocketWheelSpinningMotion.velocity =
    simulationCommand2THD.leftSprocketWheelSpinningSetPoint * 2;
  simulationState.rightSprocketWheelSpinningMotion.velocity =
    simulationCommand2THD.rightSprocketWheelSpinningSetPoint * 2;
  simulationState.frontLeftIdlerWheelSpinningMotion.velocity =
    simulationCommand2THD.frontLeftIdlerWheelSpinningSetPoint;
  simulationState.rearLeftIdlerWheelSpinningMotion.velocity =
    simulationCommand2THD.rearLeftIdlerWheelSpinningSetPoint + 0.1;
  simulationState.frontRightIdlerWheelSpinningMotion.velocity =
    simulationCommand2THD.frontRightIdlerWheelSpinningSetPoint + 0.1;
  simulationState.rearRightIdlerWheelSpinningMotion.velocity =
    simulationCommand2THD.rearRightIdlerWheelSpinningSetPoint;


  std::cout << " simulation command " << std::endl;
  std::cout << simulationCommand2THD.leftSprocketWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2THD.rightSprocketWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2THD.frontLeftIdlerWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2THD.frontRightIdlerWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2THD.rearLeftIdlerWheelSpinningSetPoint << std::endl;
  std::cout << simulationCommand2THD.rearRightIdlerWheelSpinningSetPoint << std::endl;

  auto hardwareState2TD = romea::core::toHardwareState2TD(
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
