// std
#include <ostream>

// romea
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS2FWD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FAS2FWD &frame)
{
  s << "frontLeftWheelLinearSpeed : " << frame.frontLeftWheelLinearSpeed << std::endl;
  s << "frontRightWheelLinearSpeed : "<< frame.frontRightWheelLinearSpeed << std::endl;
  s << "frontAxleSteeringAngle : " << frame.frontAxleSteeringAngle << std::endl;
  return s;
}

}  // namespace romea
