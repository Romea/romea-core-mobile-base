// std
#include <ostream>

// romea
#include "romea_core_mobile_base/odometry/OdometryFrame1FWS2RWD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FWS2RWD &frame)
{
  s << "frontWheelSteeringAngle : " << frame.frontWheelSteeringAngle << std::endl;
  s << "rearLeftWheelLinearSpeed : " << frame.rearLeftWheelLinearSpeed << std::endl;
  s << "rearRightWheelLinearSpeed : " << frame.rearRightWheelLinearSpeed << std::endl;
  return s;
}


}  // namespace romea
