// Copyright 2026 INRAE, French National Research Institute for Agriculture,
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

#ifndef ROMEA_CORE_MOBILE_BASE__INFO__THREEPOINTHITCHGEOMETRY_HPP_
#define ROMEA_CORE_MOBILE_BASE__INFO__THREEPOINTHITCHGEOMETRY_HPP_

namespace romea
{
namespace core
{

struct ThreePointHitchGeometry
{
  double lower_link_length;
  double lower_links_base_spacing;
  double cylinders_x_offset;
  double cylinders_z_offset;
  double cylinders_base_spacing;
  double cylinders_rod_attachment_distance;
  double cylinders_rod_attachment_height;
  double cylinders_dead_length;
  double cylinders_stroke;
  double upper_link_x_offset;
  double upper_link_z_offset;
  double upper_link_length;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__INFO__THREEPOINTHITCHGEOMETRY_HPP_
