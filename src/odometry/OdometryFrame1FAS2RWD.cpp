// std
#include <ostream>

// romea
#include "romea_core_mobile_base/odometry/OdometryFrame1FAS2RWD.hpp"

namespace romea {

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame1FAS2RWD &frame)
{
  s << "rearLeftWheelLinearSpeed : " << frame.rearLeftWheelLinearSpeed << std::endl;
  s << "rearRightWheelLinearSpeed : " << frame.rearRightWheelLinearSpeed << std::endl;
  s << "frontAxleSteeringAngle : " << frame.frontAxleSteeringAngle << std::endl;
  return s;
}

}  // namespace romea
