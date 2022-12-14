#ifndef ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME2FWS2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME2FWS2RWD_HPP_

// stl
#include <ostream>

namespace romea {

struct OdometryFrame2FWS2RWD
{
  double rearLeftWheelLinearSpeed;
  double frontLeftWheelSteeringAngle;
  double rearRightWheelLinearSpeed;
  double frontRightWheelSteeringAngle;

};

std::ostream & operator<<(std::ostream &s, const OdometryFrame2FWS2RWD &frame);

}  // namespace romea

#endif  / ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME2FWS2RWD_HPP_
