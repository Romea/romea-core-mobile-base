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
#include <limits>

// romea
#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/InverseSkidSteeringKinematic.hpp"


//-----------------------------------------------------------------------------
inline void testInverseForward2WD(
  const romea::core::SkidSteeringKinematic::Parameters & parameters,
  const romea::core::SkidSteeringCommandLimits & userLimits)
{
  for (size_t i = 0; i < 21; i++) {
    double linearSpeed = -1 + i * 0.1;
    for (size_t j = 0; j < 21; j++) {
      double angularSpeed = -0.5 + j * 0.05;

      romea::core::SkidSteeringCommand commandFrame;
      commandFrame.longitudinalSpeed = linearSpeed;
      commandFrame.angularSpeed = angularSpeed;

      romea::core::SkidSteeringCommand clampedCommandFrame =
        romea::core::clamp(parameters, userLimits, commandFrame);

      ASSERT_LE(
        clampedCommandFrame.longitudinalSpeed,
        userLimits.longitudinalSpeed.upper());
      ASSERT_GE(
        clampedCommandFrame.longitudinalSpeed,
        userLimits.longitudinalSpeed.lower());
      ASSERT_LE(
        std::abs(clampedCommandFrame.angularSpeed),
        userLimits.angularSpeed.upper());

      romea::core::OdometryFrame2WD odometryFrame;
      romea::core::forwardKinematic(parameters, clampedCommandFrame, odometryFrame);

      ASSERT_LE(
        std::abs(odometryFrame.leftWheelLinearSpeed),
        parameters.maximalWheelLinearSpeed);
      ASSERT_LE(
        std::abs(odometryFrame.rightWheelLinearSpeed),
        parameters.maximalWheelLinearSpeed);

      if (userLimits.longitudinalSpeed.upper() >= std::numeric_limits<double>::max() &&
        userLimits.longitudinalSpeed.lower() <= std::numeric_limits<double>::min() &&
        std::abs(commandFrame.longitudinalSpeed - clampedCommandFrame.longitudinalSpeed) >
        std::numeric_limits<double>::epsilon())
      {
        ASSERT_EQ(
          romea::core::near(
            std::abs(odometryFrame.leftWheelLinearSpeed),
            parameters.maximalWheelLinearSpeed, 0.001) ||
          romea::core::near(
            std::abs(odometryFrame.rightWheelLinearSpeed),
            parameters.maximalWheelLinearSpeed, 0.001), true);
      }

      romea::core::SkidSteeringMeasure kinematicMeasure;
      romea::core::inverseKinematic(parameters, odometryFrame, kinematicMeasure);

      ASSERT_NEAR(
        clampedCommandFrame.longitudinalSpeed,
        kinematicMeasure.longitudinalSpeed, 0.001);
      ASSERT_NEAR(
        clampedCommandFrame.angularSpeed,
        kinematicMeasure.angularSpeed, 0.001);
    }
  }
}

//-----------------------------------------------------------------------------
inline void testCinematicClamp(
  const romea::core::SkidSteeringKinematic::Parameters & parameters,
  const romea::core::SkidSteeringCommandLimits & userLimits)
{
  double dt = 0.1;
  for (size_t i = 0; i < 21; i++) {
    double firstLinearSpeed = -1 + i * 0.1;
    for (size_t j = 0; j < 21; j++) {
      double firstAngularSpeed = -0.5 + j * 0.05;
      for (size_t k = 0; k < 21; k++) {
        double secondLinearSpeed = -1 + k * 0.1;
        for (size_t l = 0; l < 21; l++) {
          double secondAngularSpeed = -0.5 + l * 0.05;

          romea::core::SkidSteeringCommand firstCommandFrame;
          firstCommandFrame.longitudinalSpeed = firstLinearSpeed;
          firstCommandFrame.angularSpeed = firstAngularSpeed;

          romea::core::SkidSteeringCommand firstClampedCommandFrame =
            romea::core::clamp(parameters, userLimits, firstCommandFrame);

          romea::core::SkidSteeringCommand secondCommandFrame;
          secondCommandFrame.longitudinalSpeed = secondLinearSpeed;
          secondCommandFrame.angularSpeed = secondAngularSpeed;

          romea::core::SkidSteeringCommand secondClampedCommandFrame =
            romea::core::clamp(parameters, userLimits, secondCommandFrame);

          secondClampedCommandFrame =
            romea::core::clamp(parameters, firstClampedCommandFrame, secondClampedCommandFrame, dt);

          ASSERT_LE(
            secondClampedCommandFrame.longitudinalSpeed,
            userLimits.longitudinalSpeed.upper());
          ASSERT_GE(
            secondClampedCommandFrame.longitudinalSpeed,
            userLimits.longitudinalSpeed.lower());
          ASSERT_LE(
            std::abs(secondClampedCommandFrame.angularSpeed),
            userLimits.angularSpeed.upper());

          romea::core::OdometryFrame2WD firstOdometryFrame;
          romea::core::forwardKinematic(parameters, firstClampedCommandFrame, firstOdometryFrame);

          romea::core::OdometryFrame2WD secondOdometryFrame;
          romea::core::forwardKinematic(parameters, secondClampedCommandFrame, secondOdometryFrame);

          ASSERT_LE(
            std::abs(
              (secondOdometryFrame.leftWheelLinearSpeed -
              firstOdometryFrame.leftWheelLinearSpeed)),
            parameters.maximalWheelLinearAcceleration * dt + 0.000001);
          ASSERT_LE(
            std::abs(
              (secondOdometryFrame.rightWheelLinearSpeed -
              firstOdometryFrame.rightWheelLinearSpeed)),
            parameters.maximalWheelLinearAcceleration * dt + 0.000001);
        }
      }
    }
  }
}


TEST(testInverseForward2D, NoLimit)
{
  romea::core::SkidSteeringKinematic::Parameters parameters;
  parameters.wheelTrack = 0.6;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  testInverseForward2WD(parameters, romea::core::SkidSteeringCommandLimits());
}

TEST(testInverseForward2D, MechanicalLimits)
{
  romea::core::SkidSteeringKinematic::Parameters parameters;
  parameters.wheelTrack = 2.5;
  parameters.maximalWheelLinearSpeed = 1;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  testInverseForward2WD(parameters, romea::core::SkidSteeringCommandLimits());
}

TEST(testInverseForward2D, UserLimits)
{
  romea::core::SkidSteeringKinematic::Parameters parameters;
  parameters.wheelTrack = 2.5;
  parameters.maximalWheelLinearSpeed = 1;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  testInverseForward2WD(parameters, romea::core::SkidSteeringCommandLimits(-0.4, 0.8, 0.5));
}

TEST(testCinematicClamp, MechanicalLimits)
{
  romea::core::SkidSteeringKinematic::Parameters parameters;
  parameters.wheelTrack = 2.5;
  parameters.maximalWheelLinearSpeed = 1;
  parameters.maximalWheelLinearAcceleration = 1;
  testCinematicClamp(parameters, romea::core::SkidSteeringCommandLimits());
}

TEST(testCinematicClamp, UserLimits)
{
  romea::core::SkidSteeringKinematic::Parameters parameters;
  parameters.wheelTrack = 2.5;
  parameters.maximalWheelLinearSpeed = 1;
  parameters.maximalWheelLinearAcceleration = 1;
  testCinematicClamp(parameters, romea::core::SkidSteeringCommandLimits(-0.4, 0.8, 0.5));
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
