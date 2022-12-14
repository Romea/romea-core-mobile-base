#ifndef ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME1FWS2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME1FWS2RWD_HPP_

// stl
#include <ostream>

namespace romea {

struct OdometryFrame1FWS2RWD
{
  double rearLeftWheelLinearSpeed;
  double rearRightWheelLinearSpeed;
  double frontWheelSteeringAngle;
};

std::ostream & operator<<(std::ostream &s, const OdometryFrame1FWS2RWD &frame);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME1FWS2RWD_HPP_
