// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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
