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


#include "romea_core_mobile_base/info/MobileBaseType.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
std::string get_kinematic_type(const std::string & mobile_base_type)
{
  if (mobile_base_type.find("1FAS") != std::string::npos) {
    return "one_axle_steering";
  } else if (mobile_base_type.find("2AS") != std::string::npos) {
    return "two_axle_steering";
  } else if (mobile_base_type.find("2FWS") != std::string::npos) {
    return "two_wheel_steering";
  } else if (mobile_base_type.find("4WS") != std::string::npos) {
    return "four_wheel_steering";
  } else if (mobile_base_type.find("4WMD") != std::string::npos) {
    return "omni_steering";
  } else if (mobile_base_type.find("2WD") != std::string::npos ||
    mobile_base_type.find("4WD") != std::string::npos ||
    mobile_base_type.find("2T") != std::string::npos)
  {
    return "skid_steering";
  } else {
    throw std::runtime_error("Mobile base type " + mobile_base_type + " not available");
  }
}

//-----------------------------------------------------------------------------
std::string get_command_type(const std::string & mobile_base_type)
{
  std::string kinematic_type = get_kinematic_type(mobile_base_type);
  if (kinematic_type.find("four_wheel_steering") != std::string::npos) {
    return "two_axle_steering";
  } else if (kinematic_type.find("two_wheel_steering") != std::string::npos) {
    return "one_axle_steering";
  } else {
    return kinematic_type;
  }
}

}  // namespace core
}  // namespace romea
