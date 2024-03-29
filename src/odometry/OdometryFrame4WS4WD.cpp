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


// std
#include <ostream>

// romea
#include "romea_core_mobile_base/odometry/OdometryFrame4WS4WD.hpp"

namespace romea
{
namespace core
{

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame4WS4WD & frame)
{
  s << "frontLeftWheelLinearSpeed : " << frame.frontLeftWheelLinearSpeed << std::endl;
  s << "frontLeftWheelSteeringAngle : " << frame.frontLeftWheelSteeringAngle << std::endl;
  s << "frontRightWheelLinearSpeed : " << frame.frontRightWheelLinearSpeed << std::endl;
  s << "frontRightWheelSteeringAngle : " << frame.frontRightWheelSteeringAngle << std::endl;
  s << "rearLeftWheelLinearSpeed : " << frame.rearLeftWheelLinearSpeed << std::endl;
  s << "rearLeftWheelSteeringAngle : " << frame.rearLeftWheelSteeringAngle << std::endl;
  s << "rearRightWheelLinearSpeed : " << frame.rearRightWheelLinearSpeed << std::endl;
  s << "rearRightWheelSteeringAngle : " << frame.rearRightWheelSteeringAngle << std::endl;
  return s;
}

}  // namespace core
}  // namespace romea
