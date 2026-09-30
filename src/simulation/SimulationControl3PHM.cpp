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

// std
#include <algorithm>
#include <cmath>
#include <stdexcept>

// romea
#include "romea_core_mobile_base/simulation/SimulationControl3PHM.hpp"

namespace {

//-----------------------------------------------------------------------------
double compute_virtual_lift_arm_length(double lower_link_length,
                                       double lower_links_base_spacing,
                                       double implement_pivot_spacing) {
  const double lateral_offset =
      0.5 * (implement_pivot_spacing - lower_links_base_spacing);
  const double squared_length =
      lower_link_length * lower_link_length - lateral_offset * lateral_offset;

  if (squared_length < 0.) {
    throw std::domain_error(
        "Invalid three-point hitch geometry: "
        "lower link length is smaller than its lateral offset");
  }

  return std::sqrt(squared_length);
}

//-----------------------------------------------------------------------------
double compute_cylinders_rod_spacing(double implement_pivot_spacing,
                                     double lower_links_base_spacing,
                                     double lower_link_length,
                                     double cylinders_rod_attachment_distance) {
  if (lower_link_length <= 0.) {
    throw std::domain_error(
        "Invalid three-point hitch geometry: lower link length is not "
        "positive");
  }

  return lower_links_base_spacing +
         cylinders_rod_attachment_distance / lower_link_length *
             (implement_pivot_spacing - lower_links_base_spacing);
}

//-----------------------------------------------------------------------------
double compute_lift_angle(double cylinder_length, double cylinders_x_offset,
                          double cylinders_z_offset,
                          double cylinders_base_spacing,
                          double cylinders_rod_spacing,
                          double lower_links_base_spacing,
                          double cylinders_rod_attachment_distance,
                          double cylinders_rod_attachment_height) {
  const double rod_lateral_offset =
      0.5 * (cylinders_rod_spacing - lower_links_base_spacing);
  const double rod_attachment_distance_xz_squared =
      cylinders_rod_attachment_distance * cylinders_rod_attachment_distance -
      rod_lateral_offset * rod_lateral_offset;

  if (rod_attachment_distance_xz_squared < 0.) {
    throw std::domain_error(
        "Invalid three-point hitch geometry: "
        "rod attachment distance is smaller than its lateral offset");
  }

  const double rod_attachment_distance_xz =
      std::sqrt(rod_attachment_distance_xz_squared);
  const double rod_attachment_radius_xz =
      std::hypot(rod_attachment_distance_xz, cylinders_rod_attachment_height);
  const double rod_attachment_angle =
      std::atan2(cylinders_rod_attachment_height, rod_attachment_distance_xz);

  const double cylinder_lateral_offset =
      0.5 * (cylinders_base_spacing - cylinders_rod_spacing);
  const double cylinder_length_xz_squared =
      cylinder_length * cylinder_length -
      cylinder_lateral_offset * cylinder_lateral_offset;

  if (cylinder_length_xz_squared < 0.) {
    throw std::domain_error(
        "Invalid cylinder length: cylinder is shorter than its lateral offset");
  }

  const double cylinder_length_xz = std::sqrt(cylinder_length_xz_squared);
  const double cylinder_base_radius_xz =
      std::hypot(cylinders_x_offset, cylinders_z_offset);

  if (cylinder_base_radius_xz == 0. || rod_attachment_radius_xz == 0.) {
    throw std::domain_error("Invalid three-point hitch geometry: zero radius");
  }

  const double cylinder_base_angle =
      std::atan2(cylinders_z_offset, cylinders_x_offset);
  double cos_triangle_angle =
      (rod_attachment_radius_xz * rod_attachment_radius_xz +
       cylinder_base_radius_xz * cylinder_base_radius_xz -
       cylinder_length_xz * cylinder_length_xz) /
      (2. * rod_attachment_radius_xz * cylinder_base_radius_xz);

  cos_triangle_angle = std::clamp(cos_triangle_angle, -1., 1.);

  const double triangle_angle = std::acos(cos_triangle_angle);
  if (cylinders_z_offset > 0.) {
    return cylinder_base_angle - rod_attachment_angle - triangle_angle;
  }

  return cylinder_base_angle - rod_attachment_angle + triangle_angle;
}

//-----------------------------------------------------------------------------
double compute_lift_cylinder_length(
    double lift_angle, double cylinders_x_offset, double cylinders_z_offset,
    double cylinders_base_spacing, double cylinders_rod_spacing,
    double lower_links_base_spacing, double cylinders_rod_attachment_distance,
    double cylinders_rod_attachment_height) {
  const double rod_lateral_offset =
      0.5 * (cylinders_rod_spacing - lower_links_base_spacing);
  const double rod_attachment_distance_xz_squared =
      cylinders_rod_attachment_distance * cylinders_rod_attachment_distance -
      rod_lateral_offset * rod_lateral_offset;

  if (rod_attachment_distance_xz_squared < 0.) {
    throw std::domain_error(
        "Invalid three-point hitch geometry: "
        "rod attachment distance is smaller than its lateral offset");
  }

  const double rod_attachment_distance_xz =
      std::sqrt(rod_attachment_distance_xz_squared);
  const double rod_attachment_radius_xz =
      std::hypot(rod_attachment_distance_xz, cylinders_rod_attachment_height);
  const double rod_attachment_angle =
      std::atan2(cylinders_rod_attachment_height, rod_attachment_distance_xz);
  const double rod_angle = lift_angle + rod_attachment_angle;
  const double rod_x = rod_attachment_radius_xz * std::cos(rod_angle);
  const double rod_z = rod_attachment_radius_xz * std::sin(rod_angle);
  const double cylinder_length_xz =
      std::hypot(rod_x - cylinders_x_offset, rod_z - cylinders_z_offset);
  const double cylinder_lateral_offset =
      0.5 * (cylinders_base_spacing - cylinders_rod_spacing);

  return std::hypot(cylinder_length_xz, cylinder_lateral_offset);
}

//-----------------------------------------------------------------------------
double compute_tilt_angle(double lift_angle, double virtual_lift_arm_length,
                          double upper_link_x_offset,
                          double upper_link_z_offset, double upper_link_length,
                          double implement_upper_link_attachment_x,
                          double implement_upper_link_attachment_z) {
  const double implement_attachment_radius = std::hypot(
      implement_upper_link_attachment_x, implement_upper_link_attachment_z);

  if (implement_attachment_radius <= 0.) {
    throw std::domain_error(
        "Invalid three-point hitch geometry: "
        "implement upper-link attachment radius must be positive");
  }

  const double implement_attachment_angle = std::atan2(
      implement_upper_link_attachment_z, implement_upper_link_attachment_x);
  const double tilt_pivot_x = virtual_lift_arm_length * std::cos(lift_angle);
  const double tilt_pivot_z = virtual_lift_arm_length * std::sin(lift_angle);
  const double dx = upper_link_x_offset - tilt_pivot_x;
  const double dz = upper_link_z_offset - tilt_pivot_z;
  const double base_distance = std::hypot(dx, dz);

  if (base_distance <= 0.) {
    throw std::domain_error(
        "Invalid three-point hitch geometry: "
        "tilt pivot and upper-link base attachment are coincident");
  }

  if (upper_link_length > base_distance + implement_attachment_radius ||
      upper_link_length <
          std::abs(base_distance - implement_attachment_radius)) {
    throw std::domain_error(
        "Invalid three-point hitch geometry: "
        "upper-link triangle cannot be closed");
  }

  const double base_angle = std::atan2(dz, dx);
  double cos_triangle_angle =
      (base_distance * base_distance +
       implement_attachment_radius * implement_attachment_radius -
       upper_link_length * upper_link_length) /
      (2. * base_distance * implement_attachment_radius);

  cos_triangle_angle = std::clamp(cos_triangle_angle, -1., 1.);

  const double absolute_attachment_angle =
      base_angle - std::acos(cos_triangle_angle);

  return absolute_attachment_angle - lift_angle - implement_attachment_angle;
}

//-----------------------------------------------------------------------------
double average(const double left, const double right) {
  return 0.5 * (left + right);
}

//-----------------------------------------------------------------------------
double compute_lift_angle(
    const romea::core::ThreePointHitchGeometry& three_point_hitch_geometry,
    const double implement_pivot_spacing, const double cylinder_length) {
  const double cylinders_rod_spacing = compute_cylinders_rod_spacing(
      implement_pivot_spacing,
      three_point_hitch_geometry.lower_links_base_spacing,
      three_point_hitch_geometry.lower_link_length,
      three_point_hitch_geometry.cylinders_rod_attachment_distance);

  return compute_lift_angle(
      cylinder_length, three_point_hitch_geometry.cylinders_x_offset,
      three_point_hitch_geometry.cylinders_z_offset,
      three_point_hitch_geometry.cylinders_base_spacing, cylinders_rod_spacing,
      three_point_hitch_geometry.lower_links_base_spacing,
      three_point_hitch_geometry.cylinders_rod_attachment_distance,
      three_point_hitch_geometry.cylinders_rod_attachment_height);
}

//-----------------------------------------------------------------------------
double compute_lift_cylinder_length(
    const romea::core::ThreePointHitchGeometry& three_point_hitch_geometry,
    const double implement_pivot_spacing, const double lift_angle) {
  const double cylinders_rod_spacing = compute_cylinders_rod_spacing(
      implement_pivot_spacing,
      three_point_hitch_geometry.lower_links_base_spacing,
      three_point_hitch_geometry.lower_link_length,
      three_point_hitch_geometry.cylinders_rod_attachment_distance);

  return compute_lift_cylinder_length(
      lift_angle, three_point_hitch_geometry.cylinders_x_offset,
      three_point_hitch_geometry.cylinders_z_offset,
      three_point_hitch_geometry.cylinders_base_spacing, cylinders_rod_spacing,
      three_point_hitch_geometry.lower_links_base_spacing,
      three_point_hitch_geometry.cylinders_rod_attachment_distance,
      three_point_hitch_geometry.cylinders_rod_attachment_height);
}

//-----------------------------------------------------------------------------
double compute_tilt_angle(
    const romea::core::ThreePointHitchGeometry& three_point_hitch_geometry,
    const romea::core::ImplementGeometry& implement_geometry,
    const double lift_angle) {
  const double virtual_lift_arm_length = compute_virtual_lift_arm_length(
      three_point_hitch_geometry.lower_link_length,
      three_point_hitch_geometry.lower_links_base_spacing,
      implement_geometry.pivot_spacing);

  return compute_tilt_angle(lift_angle, virtual_lift_arm_length,
                            three_point_hitch_geometry.upper_link_x_offset,
                            three_point_hitch_geometry.upper_link_z_offset,
                            three_point_hitch_geometry.upper_link_length,
                            implement_geometry.upper_link_attachment_x,
                            implement_geometry.upper_link_attachment_z);
}

}  // namespace

namespace romea {
namespace core {

//-----------------------------------------------------------------------------
SimulationHitchCommand3PHM toSimulationHitchCommand3PHM(
    const ThreePointHitchGeometry& three_point_hitch_geometry,
    const std::optional<ImplementGeometry>& implement_geometry,
    const HardwareCommand3PHM& hardware_command) {
  const double cylinder_length =
      average(hardware_command.left_lift_cylinder_joint,
              hardware_command.right_lift_cylinder_joint);
  const double pivot_spacing =
      implement_geometry ? implement_geometry->pivot_spacing
                         : three_point_hitch_geometry.cylinders_base_spacing;
  const double lift_angle = compute_lift_angle(
      three_point_hitch_geometry, pivot_spacing, cylinder_length);

  return {lift_angle};
}

//-----------------------------------------------------------------------------
SimulationImplementCommand3PHM toSimulationImplementCommand3PHM(
    const ThreePointHitchGeometry& three_point_hitch_geometry,
    const ImplementGeometry& implement_geometry,
    const SimulationHitchCommand3PHM& hitch_command) {
  const double tilt_angle = compute_tilt_angle(three_point_hitch_geometry,
                                               implement_geometry,
                                               hitch_command.lift_revolute_joint);

  return {tilt_angle};
}

//-----------------------------------------------------------------------------
SimulationHitchState3PHM toSimulationHitchState3PHM(
    const ThreePointHitchGeometry& three_point_hitch_geometry,
    const std::optional<ImplementGeometry>& implement_geometry,
    const HardwareState3PHM& hardware_state) {
  const auto& left_cylinder = hardware_state.left_lift_cylinder_joint;
  const auto& right_cylinder = hardware_state.right_lift_cylinder_joint;

  const double cylinder_length =
      average(left_cylinder.position, right_cylinder.position);
  const double pivot_spacing =
      implement_geometry ? implement_geometry->pivot_spacing
                         : three_point_hitch_geometry.cylinders_base_spacing;
  const double lift_angle = compute_lift_angle(
      three_point_hitch_geometry, pivot_spacing, cylinder_length);

  return {{lift_angle, 0., 0.}};
}

//-----------------------------------------------------------------------------
SimulationImplementState3PHM toSimulationImplementState3PHM(
    const ThreePointHitchGeometry& three_point_hitch_geometry,
    const ImplementGeometry& implement_geometry,
    const SimulationHitchState3PHM& hitch_state) {
  const double tilt_angle = compute_tilt_angle(three_point_hitch_geometry,
                                               implement_geometry,
                                               hitch_state.lift_revolute_joint.position);

  return {{tilt_angle, 0., 0.}};
}

//-----------------------------------------------------------------------------
HardwareState3PHM toHardwareState3PHM(
    const ThreePointHitchGeometry& three_point_hitch_geometry,
    const std::optional<ImplementGeometry>& implement_geometry,
    const SimulationHitchState3PHM& hitch_state) {
  const double pivot_spacing =
      implement_geometry ? implement_geometry->pivot_spacing
                         : three_point_hitch_geometry.cylinders_base_spacing;
  const double lift_angle = hitch_state.lift_revolute_joint.position;
  const double cylinder_length = compute_lift_cylinder_length(
      three_point_hitch_geometry, pivot_spacing, lift_angle);
  const LinearMotionState cylinder_state(cylinder_length, 0., 0.);

  return {cylinder_state, cylinder_state};
}

}  // namespace core
}  // namespace romea
