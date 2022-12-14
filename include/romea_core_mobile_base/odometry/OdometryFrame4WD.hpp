#ifndef ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME4WD_HPP_
#define ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME4WD_HPP_

// stl
#include <ostream>

namespace romea {

struct OdometryFrame4WD
{
  double frontLeftWheelLinearSpeed;
  double frontRightWheelLinearSpeed;
  double rearLeftWheelLinearSpeed;
  double rearRightWheelLinearSpeed;
};

std::ostream & operator<<(std::ostream &s, const OdometryFrame4WD &frame);

}  // namespace romea

#endif  /: ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME4WD_HPP_
