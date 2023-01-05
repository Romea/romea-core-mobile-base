// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2FWS2FWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2FWS2FWD_HPP_

// stl
#include <ostream>

namespace romea
{

struct OdometryFrame2FWS2FWD
{
  double frontLeftWheelLinearSpeed;
  double frontLeftWheelSteeringAngle;
  double frontRightWheelLinearSpeed;
  double frontRightWheelSteeringAngle;
};

std::ostream & operator<<(std::ostream & s, const OdometryFrame2FWS2FWD & frame);

}  // namespace romea

#endif // ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2FWS2FWD_HPP_
