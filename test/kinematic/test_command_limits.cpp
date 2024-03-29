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
#include "test_utils.hpp"

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp"


TEST(TestCommandLimits, SymmetricLimits)
{
  romea::core::Interval1D<double> limits = romea::core::makeSymmetricCommandLimits(1.);
  EXPECT_TRUE(limits.inside(-0.5));
  EXPECT_FALSE(limits.inside(-1.1));
  EXPECT_FALSE(limits.inside(1.1));
}

TEST(TestCommandLimits, LongitudinalSpeedLimits)
{
  EXPECT_THROW(romea::core::makeLongitudinalSpeedCommandLimits(-1, -1), std::runtime_error);
  EXPECT_THROW(romea::core::makeLongitudinalSpeedCommandLimits(1, 1), std::runtime_error);

  romea::core::Interval1D<double> limits = romea::core::makeLongitudinalSpeedCommandLimits(-0.5, 1);
  EXPECT_TRUE(limits.inside(-0.2));
  EXPECT_TRUE(limits.inside(0.7));
  EXPECT_FALSE(limits.inside(-0.6));
  EXPECT_FALSE(limits.inside(1.1));
}

TEST(TestCommandLimits, SteeringAngleSpeedLimits)
{
  EXPECT_THROW(romea::core::makeSteeringAngleCommandLimits(M_PI), std::runtime_error);
  EXPECT_THROW(romea::core::makeSteeringAngleCommandLimits(-M_PI), std::runtime_error);

  romea::core::Interval1D<double> limits = romea::core::makeSteeringAngleCommandLimits(0.7);
  EXPECT_TRUE(limits.inside(-0.1));
  EXPECT_TRUE(limits.inside(0.4));
  EXPECT_FALSE(limits.inside(-0.8));
  EXPECT_FALSE(limits.inside(0.9));
}

TEST(TestCommandLimits, OneAxleSteeringLimits)
{
  EXPECT_THROW(romea::core::OneAxleSteeringCommandLimits(1, 1, 1), std::runtime_error);
  EXPECT_THROW(romea::core::OneAxleSteeringCommandLimits(-1, -1, 1), std::runtime_error);
  EXPECT_THROW(romea::core::OneAxleSteeringCommandLimits(-1, 1, M_PI), std::runtime_error);
  EXPECT_THROW(romea::core::OneAxleSteeringCommandLimits(-1, 1, -M_PI), std::runtime_error);
  EXPECT_FALSE(romea::core::OneAxleSteeringCommandLimits().steeringAngle.inside(M_PI));
}

TEST(TestCommandLimits, ClampOneAxleSteeringCommand)
{
  romea::core::OneAxleSteeringCommandLimits limits(-1, 1, 0.5);
  romea::core::OneAxleSteeringCommand command(-2, 1.5);
  romea::core::OneAxleSteeringCommand clamped_command = clamp(command, limits);
  EXPECT_DOUBLE_EQ(clamped_command.longitudinalSpeed, limits.longitudinalSpeed.lower());
  EXPECT_DOUBLE_EQ(clamped_command.steeringAngle, limits.steeringAngle.upper());
}

TEST(TestCommandLimits, TwoAxleSteeringLimits)
{
  EXPECT_THROW(romea::core::TwoAxleSteeringCommandLimits(1, 1, 1, 1), std::runtime_error);
  EXPECT_THROW(romea::core::TwoAxleSteeringCommandLimits(-1, -1, 1, 1), std::runtime_error);
  EXPECT_THROW(romea::core::TwoAxleSteeringCommandLimits(-1, 1, M_PI, 1), std::runtime_error);
  EXPECT_THROW(romea::core::TwoAxleSteeringCommandLimits(-1, 1, -M_PI, 1), std::runtime_error);
  EXPECT_THROW(romea::core::TwoAxleSteeringCommandLimits(-1, 1, 1, M_PI), std::runtime_error);
  EXPECT_THROW(romea::core::TwoAxleSteeringCommandLimits(-1, 1, -1, -M_PI), std::runtime_error);
  EXPECT_FALSE(romea::core::TwoAxleSteeringCommandLimits().frontSteeringAngle.inside(M_PI));
  EXPECT_FALSE(romea::core::TwoAxleSteeringCommandLimits().rearSteeringAngle.inside(M_PI));
}

TEST(TestCommandLimits, ClampTwoAxleSteeringCommand)
{
  romea::core::TwoAxleSteeringCommandLimits limits(-1, 1, 0.5, 0.5);
  romea::core::TwoAxleSteeringCommand command(-2, 1.5, -1.5);
  romea::core::TwoAxleSteeringCommand clamped_command = clamp(command, limits);
  EXPECT_DOUBLE_EQ(clamped_command.longitudinalSpeed, limits.longitudinalSpeed.lower());
  EXPECT_DOUBLE_EQ(clamped_command.frontSteeringAngle, limits.frontSteeringAngle.upper());
  EXPECT_DOUBLE_EQ(clamped_command.rearSteeringAngle, limits.rearSteeringAngle.lower());
}

TEST(TestCommandLimits, SkidSteeringLimits)
{
  EXPECT_THROW(romea::core::SkidSteeringCommandLimits(1, 1, 1), std::runtime_error);
  EXPECT_THROW(romea::core::SkidSteeringCommandLimits(-1, -1, 1), std::runtime_error);
}

TEST(TestCommandLimits, ClampSkidSteeringCommand)
{
  romea::core::SkidSteeringCommandLimits limits(-1, 1, 0.5);
  romea::core::SkidSteeringCommand command(-2, -3);
  romea::core::SkidSteeringCommand clamped_command = clamp(command, limits);
  EXPECT_DOUBLE_EQ(clamped_command.longitudinalSpeed, limits.longitudinalSpeed.lower());
  EXPECT_DOUBLE_EQ(clamped_command.angularSpeed, limits.angularSpeed.lower());
}

TEST(TestCommandLimits, OmniSteeringLimits)
{
  EXPECT_THROW(romea::core::OmniSteeringCommandLimits(1, 1, 1, 1), std::runtime_error);
  EXPECT_THROW(romea::core::OmniSteeringCommandLimits(-1, -1, 1, 1), std::runtime_error);
}

TEST(TestCommandLimits, ClampOmniSteeringCommand)
{
  romea::core::OmniSteeringCommandLimits limits(-1, 1, 0.5, 0.5);
  romea::core::OmniSteeringCommand command(1.5, -2, -3);
  romea::core::OmniSteeringCommand clamped_command = clamp(command, limits);
  EXPECT_DOUBLE_EQ(clamped_command.longitudinalSpeed, limits.longitudinalSpeed.upper());
  EXPECT_DOUBLE_EQ(clamped_command.lateralSpeed, limits.lateralSpeed.lower());
  EXPECT_DOUBLE_EQ(clamped_command.angularSpeed, limits.angularSpeed.lower());
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
