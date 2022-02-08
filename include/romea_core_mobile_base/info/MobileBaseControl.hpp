#ifndef romea_MobileBaseControl_hpp
#define romea_MobileBaseControl_hpp

#include <iostream>

namespace romea {

struct WheelSpeedSensor
{
  WheelSpeedSensor();
  double speedStd;
  double speedRange;
};

struct SteeringAngleSensor
{
  SteeringAngleSensor();
  double angleStd;
  double angleRange;
};

struct WheelSpeedCommandLimits
{
  WheelSpeedCommandLimits();
  double maximalSpeed;
  double maximalAcceleration;
};

struct SteeringAngleCommandLimits
{
  SteeringAngleCommandLimits();
  double maximalAngle;
  double maximalAngularSpeed;
};

struct WheelSpeedControl
{
  WheelSpeedControl();
  WheelSpeedSensor sensor;
  WheelSpeedCommandLimits command;
};

struct SteeringAngleControl
{
  SteeringAngleControl();
  SteeringAngleSensor sensor;
  SteeringAngleCommandLimits command;
};


std::ostream& operator<<(std::ostream& os, const WheelSpeedSensor & wheelSpeedSensor);
std::ostream& operator<<(std::ostream& os, const SteeringAngleSensor & steringAngleSensor);
std::ostream& operator<<(std::ostream& os, const WheelSpeedCommandLimits & wheelSpeedCommandLimits);
std::ostream& operator<<(std::ostream& os, const SteeringAngleCommandLimits & steringAngleCommandLimits);
std::ostream& operator<<(std::ostream& os, const WheelSpeedControl & wheelSpeedControl);
std::ostream& operator<<(std::ostream& os, const SteeringAngleControl & steringAngleControl);

}
#endif
