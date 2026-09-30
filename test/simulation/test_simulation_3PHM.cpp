// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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
#include "romea_core_mobile_base/simulation/SimulationControl3PHM.hpp"

namespace {

constexpr double kTolerance = 1e-12;
constexpr double kExpectedTiltAngle = -0.44190984135248046;

romea::core::ThreePointHitchGeometry make_geometry() {
  return {
      0.398,              // lower_link_length
      0.258,              // lower_links_base_spacing
      0.130,              // cylinders_x_offset
      0.510,              // cylinders_z_offset
      0.258,              // cylinders_base_spacing
      0.152,              // cylinders_rod_attachment_distance
      0.0,                // cylinders_rod_attachment_height
      0.405474289264406,  // cylinders_dead_length
      0.105,              // cylinders_stroke
      0.0,                // upper_link_x_offset
      0.448,              // upper_link_z_offset
      0.401382610485307   // upper_link_length
  };
}

romea::core::ImplementGeometry make_implement_geometry() {
  return {
      0.258,  // pivot_spacing
      0.0,    // upper_link_attachment_x
      0.5     // upper_link_attachment_z
  };
}

TEST(TestSimulation3PHM, ConvertsSimulationStateToSymmetricCylinderPositions) {
  const auto geometry = make_geometry();
  const auto implement_geometry = make_implement_geometry();
  const double lift_angle = 0.5;
  const romea::core::SimulationHitchState3PHM hitch_state = {
      {lift_angle, 1.2, 3.4}};

  const auto hardware_state =
      romea::core::toHardwareState3PHM(
          geometry, implement_geometry, hitch_state);

  EXPECT_DOUBLE_EQ(hardware_state.left_lift_cylinder_joint.position,
                   hardware_state.right_lift_cylinder_joint.position);
  EXPECT_DOUBLE_EQ(hardware_state.left_lift_cylinder_joint.velocity, 0.0);
  EXPECT_DOUBLE_EQ(hardware_state.left_lift_cylinder_joint.force, 0.0);
  EXPECT_DOUBLE_EQ(hardware_state.right_lift_cylinder_joint.velocity, 0.0);
  EXPECT_DOUBLE_EQ(hardware_state.right_lift_cylinder_joint.force, 0.0);
}

TEST(TestSimulation3PHM, ConvertsCylinderPositionWithCylindersAboveLiftArms) {
  const auto geometry = make_geometry();
  const auto implement_geometry = make_implement_geometry();
  const double lift_angle = 0.5;
  const romea::core::SimulationHitchState3PHM reference_state = {
      {lift_angle, 0.0, 0.0}};

  const auto hardware_state = romea::core::toHardwareState3PHM(
      geometry, implement_geometry, reference_state);
  const auto hitch_state = romea::core::toSimulationHitchState3PHM(
      geometry, implement_geometry, hardware_state);

  EXPECT_NEAR(hitch_state.lift_revolute_joint.position, lift_angle,
              kTolerance);
}

TEST(TestSimulation3PHM, UsesCylinderBaseSpacingWithoutImplement) {
  const auto geometry = make_geometry();
  auto implement_geometry = make_implement_geometry();
  implement_geometry.pivot_spacing = geometry.cylinders_base_spacing;
  const romea::core::SimulationHitchState3PHM hitch_state = {
      {0.5, 0.0, 0.0}};

  const auto state_without_implement =
      romea::core::toHardwareState3PHM(geometry, std::nullopt, hitch_state);
  const auto state_with_implement = romea::core::toHardwareState3PHM(
      geometry, implement_geometry, hitch_state);

  EXPECT_DOUBLE_EQ(
      state_without_implement.left_lift_cylinder_joint.position,
      state_with_implement.left_lift_cylinder_joint.position);
  EXPECT_DOUBLE_EQ(
      state_without_implement.right_lift_cylinder_joint.position,
      state_with_implement.right_lift_cylinder_joint.position);
}

TEST(TestSimulation3PHM, ComputesOnlyHitchCommandAndStateWithoutImplement) {
  const auto geometry = make_geometry();
  auto implement_geometry = make_implement_geometry();
  implement_geometry.pivot_spacing = geometry.cylinders_base_spacing;
  const romea::core::SimulationHitchState3PHM reference_state = {
      {0.5, 0.0, 0.0}};
  const auto hardware_state = romea::core::toHardwareState3PHM(
      geometry, implement_geometry, reference_state);
  const romea::core::HardwareCommand3PHM hardware_command = {
      hardware_state.left_lift_cylinder_joint.position,
      hardware_state.right_lift_cylinder_joint.position};

  const auto command_without_implement =
      romea::core::toSimulationHitchCommand3PHM(
          geometry, std::nullopt, hardware_command);
  const auto command_with_implement =
      romea::core::toSimulationHitchCommand3PHM(
          geometry, implement_geometry, hardware_command);
  const auto state_without_implement =
      romea::core::toSimulationHitchState3PHM(
          geometry, std::nullopt, hardware_state);
  const auto state_with_implement =
      romea::core::toSimulationHitchState3PHM(
          geometry, implement_geometry, hardware_state);

  EXPECT_NEAR(command_without_implement.lift_revolute_joint,
              command_with_implement.lift_revolute_joint, kTolerance);
  EXPECT_NEAR(state_without_implement.lift_revolute_joint.position,
              state_with_implement.lift_revolute_joint.position, kTolerance);
}

TEST(TestSimulation3PHM, ConvertsAverageCylinderPositionToSimulationState) {
  const auto geometry = make_geometry();
  const auto implement_geometry = make_implement_geometry();
  const double lift_angle = 0.5;
  const romea::core::SimulationHitchState3PHM reference_state = {
      {lift_angle, 0.0, 0.0}};
  auto hardware_state =
      romea::core::toHardwareState3PHM(
          geometry, implement_geometry, reference_state);

  hardware_state.left_lift_cylinder_joint.position -= 0.01;
  hardware_state.right_lift_cylinder_joint.position += 0.01;
  hardware_state.left_lift_cylinder_joint.velocity = 1.0;
  hardware_state.left_lift_cylinder_joint.force = 2.0;
  hardware_state.right_lift_cylinder_joint.velocity = 3.0;
  hardware_state.right_lift_cylinder_joint.force = 4.0;

  const auto hitch_state = romea::core::toSimulationHitchState3PHM(
      geometry, implement_geometry, hardware_state);
  const auto implement_state = romea::core::toSimulationImplementState3PHM(
      geometry, implement_geometry, hitch_state);

  EXPECT_NEAR(hitch_state.lift_revolute_joint.position, lift_angle,
              kTolerance);
  EXPECT_DOUBLE_EQ(hitch_state.lift_revolute_joint.velocity, 0.0);
  EXPECT_DOUBLE_EQ(hitch_state.lift_revolute_joint.torque, 0.0);
  EXPECT_NEAR(implement_state.tilt_revolute_joint.position,
              kExpectedTiltAngle, kTolerance);
  EXPECT_DOUBLE_EQ(implement_state.tilt_revolute_joint.velocity, 0.0);
  EXPECT_DOUBLE_EQ(implement_state.tilt_revolute_joint.torque, 0.0);
}

TEST(TestSimulation3PHM, CommandAndStateUseTheSameKinematicConversion) {
  const auto geometry = make_geometry();
  const auto implement_geometry = make_implement_geometry();
  const romea::core::SimulationHitchState3PHM reference_state = {
      {0.5, 0.0, 0.0}};
  const auto hardware_state =
      romea::core::toHardwareState3PHM(
          geometry, implement_geometry, reference_state);
  const romea::core::HardwareCommand3PHM hardware_command = {
      hardware_state.left_lift_cylinder_joint.position - 0.01,
      hardware_state.right_lift_cylinder_joint.position + 0.01};

  const auto hitch_command = romea::core::toSimulationHitchCommand3PHM(
      geometry, implement_geometry, hardware_command);
  const auto implement_command = romea::core::toSimulationImplementCommand3PHM(
      geometry, implement_geometry, hitch_command);
  const auto hitch_state = romea::core::toSimulationHitchState3PHM(
      geometry, implement_geometry, hardware_state);
  const auto implement_state = romea::core::toSimulationImplementState3PHM(
      geometry, implement_geometry, hitch_state);

  EXPECT_NEAR(hitch_command.lift_revolute_joint,
              hitch_state.lift_revolute_joint.position, kTolerance);
  EXPECT_NEAR(implement_command.tilt_revolute_joint,
              implement_state.tilt_revolute_joint.position, kTolerance);
}

}  // namespace
