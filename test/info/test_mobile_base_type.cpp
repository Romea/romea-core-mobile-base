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
#include "romea_core_mobile_base/info/MobileBaseType.hpp"

TEST(TestMobileBaseType, testGetKinematicType)
{
  EXPECT_STREQ(romea::core::get_kinematic_type("4WS4WD").c_str(), "four_wheel_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("2FWS4WD").c_str(), "two_wheel_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("2FWS2RWD").c_str(), "two_wheel_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("2FWS2FWD").c_str(), "two_wheel_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("2AS4WD").c_str(), "two_axle_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("2AS2RWD").c_str(), "two_axle_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("2AS2FWD").c_str(), "two_axle_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("1FAS4WD").c_str(), "one_axle_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("1FAS2FWD").c_str(), "one_axle_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("1FAS2RWD").c_str(), "one_axle_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("4WD").c_str(), "skid_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("2TD").c_str(), "skid_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("2THD").c_str(), "skid_steering");
  EXPECT_STREQ(romea::core::get_kinematic_type("2TTD").c_str(), "skid_steering");
}

TEST(TestMobileBaseType, testGetCommandType)
{
  EXPECT_STREQ(romea::core::get_command_type("4WS4WD").c_str(), "two_axle_steering");
  EXPECT_STREQ(romea::core::get_command_type("2FWS4WD").c_str(), "one_axle_steering");
  EXPECT_STREQ(romea::core::get_command_type("2FWS2RWD").c_str(), "one_axle_steering");
  EXPECT_STREQ(romea::core::get_command_type("2FWS2FWD").c_str(), "one_axle_steering");
  EXPECT_STREQ(romea::core::get_command_type("2AS4WD").c_str(), "two_axle_steering");
  EXPECT_STREQ(romea::core::get_command_type("2AS2RWD").c_str(), "two_axle_steering");
  EXPECT_STREQ(romea::core::get_command_type("2AS2FWD").c_str(), "two_axle_steering");
  EXPECT_STREQ(romea::core::get_command_type("1FAS4WD").c_str(), "one_axle_steering");
  EXPECT_STREQ(romea::core::get_command_type("1FAS2FWD").c_str(), "one_axle_steering");
  EXPECT_STREQ(romea::core::get_command_type("1FAS2RWD").c_str(), "one_axle_steering");
  EXPECT_STREQ(romea::core::get_command_type("4WD").c_str(), "skid_steering");
  EXPECT_STREQ(romea::core::get_command_type("2TD").c_str(), "skid_steering");
  EXPECT_STREQ(romea::core::get_command_type("2THD").c_str(), "skid_steering");
  EXPECT_STREQ(romea::core::get_command_type("2TTD").c_str(), "skid_steering");
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
