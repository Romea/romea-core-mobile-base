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


#ifndef ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASECONTROL_HPP_
#define ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASECONTROL_HPP_

#include <iostream>

namespace romea
{

struct WheelSpeedSensor
{
  double speedStd;
  double speedRange;
};

struct SteeringAngleSensor
{
  double angleStd;
  double angleRange;
};

struct WheelSpeedCommandLimits
{
  double maximalSpeed;
  double maximalAcceleration;
};

struct SteeringAngleCommandLimits
{
  double maximalAngle;
  double maximalAngularSpeed;
};

struct WheelSpeedControl
{
  WheelSpeedSensor sensor;
  WheelSpeedCommandLimits command;
};

struct SteeringAngleControl
{
  SteeringAngleSensor sensor;
  SteeringAngleCommandLimits command;
};


//std::ostream& operator<<(std::ostream& os, const WheelSpeedSensor & wheelSpeedSensor);
//std::ostream& operator<<(std::ostream& os, const SteeringAngleSensor & steringAngleSensor);
//std::ostream& operator<<(std::ostream& os, const WheelSpeedCommandLimits & wheelSpeedCommandLimits);
//std::ostream& operator<<(std::ostream& os, const SteeringAngleCommandLimits & steringAngleCommandLimits);
//std::ostream& operator<<(std::ostream& os, const WheelSpeedControl & wheelSpeedControl);
//std::ostream& operator<<(std::ostream& os, const SteeringAngleControl & steringAngleControl);

}

#endif  // ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASECONTROL_HPP_
