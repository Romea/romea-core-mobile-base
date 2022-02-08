#include "romea_core_mobile_base/info/MobileBaseControl.hpp"

#include <limits>

namespace romea {

//-----------------------------------------------------------------------------
WheelSpeedSensor::WheelSpeedSensor():
  speedStd(std::numeric_limits<double>::quiet_NaN()),
  speedRange(std::numeric_limits<double>::max())
{

}

//-----------------------------------------------------------------------------
SteeringAngleSensor::SteeringAngleSensor():
   angleStd(std::numeric_limits<double>::quiet_NaN()),
   angleRange(std::numeric_limits<double>::max())
{

}

//-----------------------------------------------------------------------------
WheelSpeedCommandLimits::WheelSpeedCommandLimits():
  maximalSpeed(0.0),
  maximalAcceleration(0.0)
{

}

//-----------------------------------------------------------------------------
SteeringAngleCommandLimits::SteeringAngleCommandLimits():
  maximalAngle(0.0),
  maximalAngularSpeed(0.0)
{

}

//-----------------------------------------------------------------------------
WheelSpeedControl::WheelSpeedControl():
  sensor(),
  command()
{
}

//-----------------------------------------------------------------------------
SteeringAngleControl::SteeringAngleControl():
  sensor(),
  command()
{

}


//std::ostream& operator<<(std::ostream& os, const WheelSpeedSensor & wheelSpeedSensor)
//{
//  os << " std " << wheelSpeedSensor.speed_std <<" "<< wheelSpeedSensor.speed_range;
//}

//std::ostream& operator<<(std::ostream& os, const SteeringAngleSensor & steringAngleSensor)
//{

//}

//std::ostream& operator<<(std::ostream& os, const WheelSpeedCommandLimits & wheelSpeedCommandLimits);
//std::ostream& operator<<(std::ostream& os, const SteeringAngleCommandLimits & steringAngleCommandLimits);
//std::ostream& operator<<(std::ostream& os, const WheelSpeedControl & wheelSpeedControl);
//std::ostream& operator<<(std::ostream& os, const SteeringAngleControl & steringAngleControl);

}
