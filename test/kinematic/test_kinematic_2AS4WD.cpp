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

// romea core
#include <romea_core_common/math/Algorithm.hpp>

// std
#include <iostream>
#include <limits>

// local
#include "test_utils.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/FowardTwoAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/InverseTwoAxleSteeringKinematic.hpp"


//-----------------------------------------------------------------------------
inline void testInverseForward2AS24WD(
  const romea::TwoAxleSteeringKinematic::Parameters & parameters,
  const romea::TwoAxleSteeringCommandLimits & userLimits)
{
  for (size_t i = 0; i < 21; i++) {
    double linearSpeed = -1 + i * 0.1;
    for (size_t j = 0; j < 21; j++) {
      double frontSteeringAngle = -0.5 + j * 0.05;

      for (size_t k = 0; k < 21; k++) {
        double rearSteeringAngle = -0.5 + k * 0.05;

        romea::TwoAxleSteeringCommand commandFrame;
        commandFrame.longitudinalSpeed = linearSpeed;
        commandFrame.frontSteeringAngle = frontSteeringAngle;
        commandFrame.rearSteeringAngle = rearSteeringAngle;

        romea::TwoAxleSteeringCommand clampedCommandFrame =
          romea::clamp(parameters, userLimits, commandFrame);

        ASSERT_LE(
          clampedCommandFrame.longitudinalSpeed,
          userLimits.longitudinalSpeed.upper());
        ASSERT_GE(
          clampedCommandFrame.longitudinalSpeed,
          userLimits.longitudinalSpeed.lower());
        ASSERT_LE(
          std::abs(clampedCommandFrame.frontSteeringAngle),
          userLimits.frontSteeringAngle.upper());
        ASSERT_LE(
          std::abs(clampedCommandFrame.rearSteeringAngle),
          userLimits.rearSteeringAngle.upper());

        romea::OdometryFrame2AS4WD odometryFrame;
        romea::forwardKinematic(parameters, clampedCommandFrame, odometryFrame);

        ASSERT_LE(
          std::abs(odometryFrame.frontLeftWheelLinearSpeed),
          parameters.frontMaximalWheelLinearSpeed);
        ASSERT_LE(
          std::abs(odometryFrame.frontRightWheelLinearSpeed),
          parameters.frontMaximalWheelLinearSpeed);
        ASSERT_LE(
          std::abs(odometryFrame.rearLeftWheelLinearSpeed),
          parameters.rearMaximalWheelLinearSpeed);
        ASSERT_LE(
          std::abs(odometryFrame.frontRightWheelLinearSpeed),
          parameters.rearMaximalWheelLinearSpeed);


        if (std::abs(
            clampedCommandFrame.frontSteeringAngle -
            clampedCommandFrame.rearSteeringAngle) <
          std::numeric_limits<double>::epsilon())
        {
          ASSERT_NEAR(
            odometryFrame.frontLeftWheelLinearSpeed,
            clampedCommandFrame.longitudinalSpeed /
            std::cos(clampedCommandFrame.frontSteeringAngle), 0.001);
          ASSERT_NEAR(
            odometryFrame.frontRightWheelLinearSpeed,
            clampedCommandFrame.longitudinalSpeed /
            std::cos(clampedCommandFrame.frontSteeringAngle), 0.001);
          ASSERT_NEAR(
            odometryFrame.rearLeftWheelLinearSpeed,
            clampedCommandFrame.longitudinalSpeed /
            std::cos(clampedCommandFrame.rearSteeringAngle), 0.001);
          ASSERT_NEAR(
            odometryFrame.frontRightWheelLinearSpeed,
            clampedCommandFrame.longitudinalSpeed /
            std::cos(clampedCommandFrame.rearSteeringAngle), 0.001);
        }

        if (userLimits.longitudinalSpeed.upper() >= std::numeric_limits<double>::max() &&
          userLimits.longitudinalSpeed.lower() <= std::numeric_limits<double>::min() &&
          std::abs(commandFrame.longitudinalSpeed - clampedCommandFrame.longitudinalSpeed) >
          std::numeric_limits<double>::epsilon())
        {
          ASSERT_EQ(
            romea::near(
              std::abs(odometryFrame.frontLeftWheelLinearSpeed),
              parameters.frontMaximalWheelLinearSpeed, 0.001) ||
            romea::near(
              std::abs(odometryFrame.frontRightWheelLinearSpeed),
              parameters.frontMaximalWheelLinearSpeed, 0.001) ||
            romea::near(
              std::abs(odometryFrame.rearLeftWheelLinearSpeed),
              parameters.rearMaximalWheelLinearSpeed, 0.001) ||
            romea::near(
              std::abs(odometryFrame.rearRightWheelLinearSpeed),
              parameters.rearMaximalWheelLinearSpeed, 0.001), true);
        }

        ASSERT_LE(
          std::abs(odometryFrame.frontAxleSteeringAngle),
          parameters.frontMaximalSteeringAngle);
        ASSERT_LE(
          std::abs(odometryFrame.rearAxleSteeringAngle),
          parameters.rearMaximalSteeringAngle);

        romea::TwoAxleSteeringMeasure kinematicMeasure;
        romea::inverseKinematic(parameters, odometryFrame, kinematicMeasure);

        ASSERT_NEAR(
          clampedCommandFrame.longitudinalSpeed,
          kinematicMeasure.longitudinalSpeed, 0.001);
        ASSERT_NEAR(
          clampedCommandFrame.frontSteeringAngle,
          kinematicMeasure.frontSteeringAngle, 0.001);
        ASSERT_NEAR(
          clampedCommandFrame.rearSteeringAngle,
          kinematicMeasure.rearSteeringAngle, 0.001);
      }
    }
  }
}

TEST(testInverseForward2AS4WD, SameTrackSameWheelbase)
{
  romea::TwoAxleSteeringCommandLimits userLimits;

  romea::TwoAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 0.7;
  parameters.rearWheelBase = 0.7;
  parameters.frontWheelTrack = 1.2;
  parameters.rearWheelTrack = 1.2;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.steeringAngleVariance = 0.02 * 0.02;

  testInverseForward2AS24WD(parameters, userLimits);
}

TEST(testInverseForward2AS4WD, DiffTrackSameWheelbase)
{
  romea::TwoAxleSteeringCommandLimits userLimits;

  romea::TwoAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 0.7;
  parameters.rearWheelBase = 0.7;
  parameters.frontWheelTrack = 1.2;
  parameters.rearWheelTrack = 1.4;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.steeringAngleVariance = 0.02 * 0.02;

  testInverseForward2AS24WD(parameters, userLimits);
}

TEST(testInverseForward2AS4WD, DiffTrackDiffWheelbase)
{
  romea::TwoAxleSteeringCommandLimits userLimits;

  romea::TwoAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1;
  parameters.rearWheelBase = 0.7;
  parameters.frontWheelTrack = 1.2;
  parameters.rearWheelTrack = 1.4;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.steeringAngleVariance = 0.02 * 0.02;

  testInverseForward2AS24WD(parameters, userLimits);
}

TEST(testInverseForward2AS24WD, HubOffset)
{
  romea::TwoAxleSteeringCommandLimits userLimits;

  romea::TwoAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1;
  parameters.rearWheelBase = 0.7;
  parameters.frontWheelTrack = 1.2;
  parameters.rearWheelTrack = 1.4;
  parameters.frontHubCarrierOffset = 0.1;
  parameters.rearHubCarrierOffset = 0.1;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.steeringAngleVariance = 0.02 * 0.02;

  testInverseForward2AS24WD(parameters, userLimits);
}

TEST(testInverseForward2AS24WD, MecanicalLimits)
{
  romea::TwoAxleSteeringCommandLimits userLimits;

  romea::TwoAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1;
  parameters.rearWheelBase = 0.7;
  parameters.frontWheelTrack = 1.2;
  parameters.rearWheelTrack = 1.4;
  parameters.frontHubCarrierOffset = 0.1;
  parameters.rearHubCarrierOffset = 0.1;
  parameters.frontMaximalWheelLinearSpeed = 1;
  parameters.rearMaximalWheelLinearSpeed = 1;
  parameters.frontMaximalSteeringAngle = 0.3;
  parameters.rearMaximalSteeringAngle = 0.2;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.steeringAngleVariance = 0.02 * 0.02;

  testInverseForward2AS24WD(parameters, userLimits);
}

TEST(testInverseForward2AS24WD, UserLimits)
{
  romea::TwoAxleSteeringCommandLimits userLimits(-0.4, 0.8, 0.25, 0.25);

  romea::TwoAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1;
  parameters.rearWheelBase = 0.7;
  parameters.frontWheelTrack = 1.2;
  parameters.rearWheelTrack = 1.4;
  parameters.frontHubCarrierOffset = 0.1;
  parameters.rearHubCarrierOffset = 0.1;
  parameters.frontMaximalWheelLinearSpeed = 1;
  parameters.rearMaximalWheelLinearSpeed = 1;
  parameters.frontMaximalSteeringAngle = 0.3;
  parameters.rearMaximalSteeringAngle = 0.2;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.steeringAngleVariance = 0.02 * 0.02;

  testInverseForward2AS24WD(parameters, userLimits);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
