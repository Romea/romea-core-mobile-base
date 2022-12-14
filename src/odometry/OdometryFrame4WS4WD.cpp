// std
#include <ostream>

// romea
#include "romea_core_mobile_base/odometry/OdometryFrame4WS4WD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame4WS4WD &frame)
{
  s << "frontLeftWheelLinearSpeed : "<< frame.frontLeftWheelLinearSpeed << std::endl;
  s << "frontLeftWheelSteeringAngle : "<< frame.frontLeftWheelSteeringAngle << std::endl;
  s << "frontRightWheelLinearSpeed : "<< frame.frontRightWheelLinearSpeed << std::endl;
  s << "frontRightWheelSteeringAngle : "<< frame.frontRightWheelSteeringAngle << std::endl;
  s << "rearLeftWheelLinearSpeed : "<< frame.rearLeftWheelLinearSpeed << std::endl;
  s << "rearLeftWheelSteeringAngle : "<< frame.rearLeftWheelSteeringAngle << std::endl;
  s << "rearRightWheelLinearSpeed : "<< frame.rearRightWheelLinearSpeed << std::endl;
  s << "rearRightWheelSteeringAngle : "<< frame.rearRightWheelSteeringAngle << std::endl;
  return s;
}

}  // namespace romea
