#ifndef ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME4WS4WD_HPP_
#define ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME4WS4WD_HPP_

// stl
#include <ostream>

namespace romea {

struct OdometryFrame4WS4WD
{
  double frontLeftWheelLinearSpeed;
  double frontLeftWheelSteeringAngle;

  double frontRightWheelLinearSpeed;
  double frontRightWheelSteeringAngle;

  double rearLeftWheelLinearSpeed;
  double rearLeftWheelSteeringAngle;

  double rearRightWheelLinearSpeed;
  double rearRightWheelSteeringAngle;
};

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_ODOMETRY_ODOMETRYFRAME4WS4WD_HPP_
