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
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/InverseTwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringMeasure.hpp"


//-----------------------------------------------------------------------------
inline void testInverseForward2FWS2FWD(
  const romea::core::TwoWheelSteeringKinematic::Parameters & parameters,
  const romea::core::OneAxleSteeringCommandLimits & userLimits)
{
  for (size_t i = 0; i < 21; i++) {
    double linearSpeed = -1 + i * 0.1;
    for (size_t j = 0; j < 21; j++) {
      double steeringAngle = -0.5 + j * 0.05;

      romea::core::OneAxleSteeringCommand commandFrame;
      commandFrame.longitudinalSpeed = linearSpeed;
      commandFrame.steeringAngle = steeringAngle;

      romea::core::OneAxleSteeringCommand clampedCommandFrame =
        romea::core::clamp(parameters, userLimits, commandFrame);

      ASSERT_LE(
        clampedCommandFrame.longitudinalSpeed,
        userLimits.longitudinalSpeed.upper());
      ASSERT_GE(
        clampedCommandFrame.longitudinalSpeed,
        userLimits.longitudinalSpeed.lower());
      ASSERT_LE(
        std::abs(clampedCommandFrame.steeringAngle),
        userLimits.steeringAngle.upper());

      romea::core::OdometryFrame2FWS2FWD odometryFrame;
      romea::core::forwardKinematic(parameters, clampedCommandFrame, odometryFrame);

      ASSERT_LE(
        std::abs(odometryFrame.frontLeftWheelLinearSpeed),
        parameters.frontMaximalWheelLinearSpeed);
      ASSERT_LE(
        std::abs(odometryFrame.frontRightWheelLinearSpeed),
        parameters.frontMaximalWheelLinearSpeed);
      ASSERT_LE(
        std::abs(odometryFrame.frontLeftWheelSteeringAngle),
        parameters.maximalWheelSteeringAngle);
      ASSERT_LE(
        std::abs(odometryFrame.frontRightWheelSteeringAngle),
        parameters.maximalWheelSteeringAngle);

      if (userLimits.longitudinalSpeed.upper() >= std::numeric_limits<double>::max() &&
        userLimits.longitudinalSpeed.lower() <= std::numeric_limits<double>::min() &&
        std::abs(commandFrame.longitudinalSpeed - clampedCommandFrame.longitudinalSpeed) >
        std::numeric_limits<double>::epsilon())
      {
        ASSERT_EQ(
          romea::core::near(
            std::abs(odometryFrame.frontLeftWheelLinearSpeed),
            parameters.frontMaximalWheelLinearSpeed, 0.001) ||
          romea::core::near(
            std::abs(odometryFrame.frontRightWheelLinearSpeed),
            parameters.frontMaximalWheelLinearSpeed, 0.001), true);
      }

      romea::core::OneAxleSteeringMeasure kinematicMeasure;
      romea::core::inverseKinematic(parameters, odometryFrame, kinematicMeasure);

      ASSERT_NEAR(
        clampedCommandFrame.longitudinalSpeed,
        kinematicMeasure.longitudinalSpeed, 0.001);
      ASSERT_NEAR(
        clampedCommandFrame.steeringAngle,
        kinematicMeasure.steeringAngle, 0.001);
    }
  }
}


//-----------------------------------------------------------------------------
inline double radius(double x, double y, double theta, double xw, double yw, double thetaw)
{
  double vx1 = std::cos(theta + M_PI / 2);
  double vy1 = std::sin(theta + M_PI / 2);
  double vx2 = std::cos(thetaw + M_PI / 2);
  double vy2 = std::sin(thetaw + M_PI / 2);
  double vx21 = (xw - x);
  double vy21 = (yw - y);

  return (vx21 * vy2 - vy21 * vx2) / (vx1 * vy2 - vy1 * vx2);
}

//-----------------------------------------------------------------------------
inline void testCircularMovement(
  romea::core::TwoWheelSteeringKinematic::Parameters & parameters,
  const double & v,
  const double & R)
{
  const double wheelBase = parameters.frontWheelBase + parameters.rearWheelBase;
  const double frontWheelTrack = parameters.frontWheelTrack;
  const double rearWheelTrack = parameters.rearWheelTrack;

  double dt = 0.0001;
  double K = 1 / R;

  size_t n = 2 * M_PI * std::abs(R) / (v * dt);

  romea::core::OneAxleSteeringCommand command;
  command.longitudinalSpeed = v;
  command.steeringAngle = std::atan(K * wheelBase);


  double deltafl, deltafr;
  double vrl, vrr, vfl, vfr;

  romea::core::OdometryFrame2FWS2FWD odometryFrame;
  romea::core::forwardKinematic(parameters, command, odometryFrame);

  deltafl = odometryFrame.frontLeftWheelSteeringAngle;
  deltafr = odometryFrame.frontRightWheelSteeringAngle;

  vfl = odometryFrame.frontLeftWheelLinearSpeed;
  vfr = odometryFrame.frontRightWheelLinearSpeed;
  double klf = std::tan(deltafl) / wheelBase;
  double alpha1 = (1 - klf * (rearWheelTrack - frontWheelTrack) / 2.);

  double krf = std::tan(deltafr) / wheelBase;
  double alpha2 = (1 + krf * (rearWheelTrack - frontWheelTrack) / 2.);

  vrl = vfl * cos(odometryFrame.frontLeftWheelSteeringAngle) * alpha1;
  vrr = vfr * cos(odometryFrame.frontRightWheelSteeringAngle) * alpha2;

  double xfl = wheelBase;
  double yfl = frontWheelTrack / 2.;
  double xfr = wheelBase;
  double yfr = -frontWheelTrack / 2.;
  double xrl = 0;
  double yrl = rearWheelTrack / 2.;
  double xrr = 0;
  double yrr = -rearWheelTrack / 2.;


  for (size_t i = 1; i < n; i++) {
    // Vehicle orientation and center
    double theta = std::atan2(
      0.5 * (yfl + yfr) - 0.5 * (yrl + yrr),
      0.5 * (xfl + xfr) - 0.5 * (xrl + xrr));
    double x = 0.5 * (xrl + xrr);
    double y = 0.5 * (yrl + yrr);

    // Wheel orientatons
    double thetarl = theta;
    double thetarr = theta;
    double thetafl = theta + deltafl;
    double thetafr = theta + deltafr;

    // New wheel positions
    xfl += std::cos(thetafl) * vfl * dt;
    yfl += std::sin(thetafl) * vfl * dt;
    xfr += std::cos(thetafr) * vfr * dt;
    yfr += std::sin(thetafr) * vfr * dt;
    xrl += std::cos(thetarl) * vrl * dt;
    yrl += std::sin(thetarl) * vrl * dt;
    xrr += std::cos(thetarr) * vrr * dt;
    yrr += std::sin(thetarr) * vrr * dt;

    // Compute radius of curvature
    double rfl = radius(x, y, theta, xfl, yfl, thetafl);
    double rfr = radius(x, y, theta, xfr, yfr, thetafr);
    ASSERT_NEAR(rfl, R, 0.001);
    ASSERT_NEAR(rfr, R, 0.001);
  }

  ASSERT_NEAR(xfl, wheelBase, 0.01);
  ASSERT_NEAR(yfl, frontWheelTrack / 2., 0.01);
  ASSERT_NEAR(xfr, wheelBase, 0.01);
  ASSERT_NEAR(yfr, -frontWheelTrack / 2., 0.01);
  ASSERT_NEAR(xrl, 0, 0.01);
  ASSERT_NEAR(yrl, rearWheelTrack / 2., 0.01);
  ASSERT_NEAR(xrr, 0, 0.01);
  ASSERT_NEAR(yrr, -rearWheelTrack / 2., 0.01);
}

TEST(testInverseForward2FWS2FWD, SameTrack)
{
  romea::core::OneAxleSteeringCommandLimits userLimits;

  romea::core::TwoWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 0.7;
  parameters.rearWheelBase = 0.7;
  parameters.frontWheelTrack = 1.2;
  parameters.rearWheelTrack = 1.2;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.wheelSteeringAngleVariance = 0.02 * 0.02;

  testInverseForward2FWS2FWD(parameters, userLimits);
}

TEST(testInverseForward2FWS2FWD, DiffTrack)
{
  romea::core::OneAxleSteeringCommandLimits userLimits;

  romea::core::TwoWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 0.7;
  parameters.rearWheelBase = 0.7;
  parameters.frontWheelTrack = 1.2;
  parameters.rearWheelTrack = 1.6;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.wheelSteeringAngleVariance = 0.02 * 0.02;

  testInverseForward2FWS2FWD(parameters, userLimits);
}

TEST(testInverseForward2FWS2FWD, HubOffset)
{
  romea::core::OneAxleSteeringCommandLimits userLimits;

  romea::core::TwoWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 0.7;
  parameters.rearWheelBase = 0.7;
  parameters.frontWheelTrack = 1.2;
  parameters.rearWheelTrack = 1.6;
  parameters.frontHubCarrierOffset = 0.1;
  parameters.rearHubCarrierOffset = 0.1;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.wheelSteeringAngleVariance = 0.02 * 0.02;

  testInverseForward2FWS2FWD(parameters, userLimits);
}

TEST(testInverseForward2FWS2FWD, MecanicalLimits)
{
  romea::core::OneAxleSteeringCommandLimits userLimits;

  romea::core::TwoWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1.25;
  parameters.rearWheelBase = 1.25;
  parameters.frontWheelTrack = 1.4;
  parameters.rearWheelTrack = 1.6;
  parameters.frontHubCarrierOffset = 0.1;
  parameters.rearHubCarrierOffset = 0.1;
  parameters.frontMaximalWheelLinearSpeed = 1;
  parameters.maximalWheelSteeringAngle = 0.3;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.wheelSteeringAngleVariance = 0.02 * 0.02;

  testInverseForward2FWS2FWD(parameters, userLimits);
}

TEST(testInverseForward2FWS2FWD, UserLimits)
{
  romea::core::OneAxleSteeringCommandLimits userLimits(-0.4, 0.8, 0.25);

  romea::core::TwoWheelSteeringKinematic::Parameters parameters;
  parameters.frontWheelBase = 1.25;
  parameters.rearWheelBase = 1.25;
  parameters.frontWheelTrack = 1.4;
  parameters.rearWheelTrack = 1.6;
  parameters.frontHubCarrierOffset = 0.1;
  parameters.rearHubCarrierOffset = 0.1;
  parameters.frontMaximalWheelLinearSpeed = 1;
  parameters.maximalWheelSteeringAngle = 0.3;
  parameters.wheelLinearSpeedVariance = 0.1 * 0.1;
  parameters.wheelSteeringAngleVariance = 0.02 * 0.02;

  testInverseForward2FWS2FWD(parameters, userLimits);
}

//-----------------------------------------------------------------------------
TEST(Test2WS, testCircularMovement2FWS2FWD)
{
  {
    romea::core::TwoWheelSteeringKinematic::Parameters parameters;
    parameters.frontWheelBase = 1.4;
    parameters.rearWheelBase = 0;
    parameters.frontWheelTrack = 1.2;
    parameters.rearWheelTrack = 1.2;
    testCircularMovement(parameters, 1, 3.3);
  }

  {
    romea::core::TwoWheelSteeringKinematic::Parameters parameters;
    parameters.frontWheelBase = 1.4;
    parameters.rearWheelBase = 0;
    parameters.frontWheelTrack = 0.8;
    parameters.rearWheelTrack = 0.8;
    testCircularMovement(parameters, 1, -3.3);
  }

  {
    romea::core::TwoWheelSteeringKinematic::Parameters parameters;
    parameters.frontWheelBase = 1.4;
    parameters.rearWheelBase = 0;
    parameters.frontWheelTrack = 0.8;
    parameters.rearWheelTrack = 1.2;
    testCircularMovement(parameters, 1, 3.3);
  }

  {
    romea::core::TwoWheelSteeringKinematic::Parameters parameters;
    parameters.frontWheelBase = 1.4;
    parameters.rearWheelBase = 0;
    parameters.frontWheelTrack = 0.8;
    parameters.rearWheelTrack = 1.2;
    testCircularMovement(parameters, 1, -3.3);
  }
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
