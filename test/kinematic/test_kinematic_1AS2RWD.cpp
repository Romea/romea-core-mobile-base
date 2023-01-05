// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// romea core
#include <romea_core_common/math/Algorithm.hpp>

// std
#include <iostream>

// local
#include "test_utils.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/InverseOneAxleSteeringKinematic.hpp"


//-----------------------------------------------------------------------------
inline void testInverseForward1FAS2RWD(
  const romea::OneAxleSteeringKinematic::Parameters & parameters,
  const romea::OneAxleSteeringCommandLimits & userLimits)
{
  for (size_t i = 0; i < 21; i++) {
    double linearSpeed = -1 + i * 0.1;
    for (size_t j = 0; j < 21; j++) {
      double steeringAngle = -0.5 + j * 0.05;

      romea::OneAxleSteeringCommand commandFrame;
      commandFrame.longitudinalSpeed = linearSpeed;
      commandFrame.steeringAngle = steeringAngle;

      romea::OneAxleSteeringCommand clampedCommandFrame =
        romea::clamp(parameters, userLimits, commandFrame);

      ASSERT_LE(
        clampedCommandFrame.longitudinalSpeed,
        userLimits.longitudinalSpeed.upper());
      ASSERT_GE(
        clampedCommandFrame.longitudinalSpeed,
        userLimits.longitudinalSpeed.lower());
      ASSERT_LE(
        std::abs(clampedCommandFrame.steeringAngle),
        userLimits.steeringAngle.upper());

      romea::OdometryFrame1FAS2RWD odometryFrame;
      romea::forwardKinematic(parameters, clampedCommandFrame, odometryFrame);

      ASSERT_LE(
        std::abs(odometryFrame.rearLeftWheelLinearSpeed),
        parameters.rearMaximalWheelLinearSpeed);
      ASSERT_LE(
        std::abs(odometryFrame.rearRightWheelLinearSpeed),
        parameters.rearMaximalWheelLinearSpeed);
      ASSERT_LE(
        std::abs(odometryFrame.frontAxleSteeringAngle),
        parameters.maximalSteeringAngle);

      romea::OneAxleSteeringMeasure kinematicMeasure;
      romea::inverseKinematic(parameters, odometryFrame, kinematicMeasure);

      ASSERT_NEAR(
        clampedCommandFrame.longitudinalSpeed,
        kinematicMeasure.longitudinalSpeed, 0.001);
      ASSERT_NEAR(
        clampedCommandFrame.steeringAngle,
        kinematicMeasure.steeringAngle, 0.001);
    }
  }
}


TEST(testInverseForward1FAS2RWD, SameTrack)
{
  romea::OneAxleSteeringCommandLimits userLimits;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 0.7;
  parameters.rearWheelBase = 0.7;
  parameters.frontWheelTrack = 1.2;
  parameters.rearWheelTrack = 1.2;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.steeringAngleVariance = 0.02 * 0.02;

  testInverseForward1FAS2RWD(parameters, userLimits);
}

TEST(testInverseForward1FAS2RWD, DiffTrack)
{
  romea::OneAxleSteeringCommandLimits userLimits;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1.25;
  parameters.rearWheelBase = 1.25;
  parameters.frontWheelTrack = 1.4;
  parameters.rearWheelTrack = 1.8;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.steeringAngleVariance = 0.02 * 0.02;

  testInverseForward1FAS2RWD(parameters, userLimits);
}


TEST(testInverseForward1FAS2RWD, HubOffset)
{
  romea::OneAxleSteeringCommandLimits userLimits;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1.25;
  parameters.rearWheelBase = 1.25;
  parameters.frontWheelTrack = 1.4;
  parameters.rearWheelTrack = 1.8;
  parameters.frontHubCarrierOffset = 0.1;
  parameters.rearHubCarrierOffset = 0.1;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.steeringAngleVariance = 0.02 * 0.02;

  testInverseForward1FAS2RWD(parameters, userLimits);
}

TEST(testInverseForward1FAS2RWD, MecanicalLimits)
{
  romea::OneAxleSteeringCommandLimits userLimits;

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1.25;
  parameters.rearWheelBase = 1.25;
  parameters.frontWheelTrack = 1.4;
  parameters.rearWheelTrack = 1.8;
  parameters.frontHubCarrierOffset = 0.1;
  parameters.rearHubCarrierOffset = 0.1;
  parameters.frontMaximalWheelLinearSpeed = 1;
  parameters.maximalSteeringAngle = 0.3;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.steeringAngleVariance = 0.02 * 0.02;

  testInverseForward1FAS2RWD(parameters, userLimits);
}

TEST(testInverseForward1FAS2RWD, UserLimits)
{
  romea::OneAxleSteeringCommandLimits userLimits(-0.4, 0.9, 0.25);

  romea::OneAxleSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1.25;
  parameters.rearWheelBase = 1.25;
  parameters.frontWheelTrack = 1.4;
  parameters.rearWheelTrack = 1.8;
  parameters.frontHubCarrierOffset = 0.1;
  parameters.rearHubCarrierOffset = 0.1;
  parameters.frontMaximalWheelLinearSpeed = 1;
  parameters.maximalSteeringAngle = 0.3;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.steeringAngleVariance = 0.02 * 0.02;

  testInverseForward1FAS2RWD(parameters, userLimits);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
