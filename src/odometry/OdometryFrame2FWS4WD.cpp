// std
#include <ostream>

// romea
#include "romea_core_mobile_base/odometry/OdometryFrame2FWS4WD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2FWS4WD &frame)
{
  s << "frontLeftWheelSteeringAngle : "<< frame.frontLeftWheelSteeringAngle << std::endl;
  s << "frontRightWheelSteeringAngle : "<< frame.frontRightWheelSteeringAngle << std::endl;
  s << "frontLeftWheelLinearSpeed : "<< frame.frontLeftWheelLinearSpeed << std::endl;
  s << "frontRightWheelLinearSpeed : "<< frame.frontRightWheelLinearSpeed << std::endl;
  s << "rearLeftWheelLinearSpeed : "<< frame.rearLeftWheelLinearSpeed << std::endl;
  s << "rearRightWheelLinearSpeed : "<< frame.rearRightWheelLinearSpeed << std::endl;
  return s;
}

}  //  namespace romea
