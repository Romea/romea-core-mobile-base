// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2FWS4WD_HPP_
#define ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2FWS4WD_HPP_

// stl
#include <ostream>

namespace romea
{

struct OdometryFrame2FWS4WD
{
  double frontLeftWheelLinearSpeed;
  double rearLeftWheelLinearSpeed;
  double frontLeftWheelSteeringAngle;
  double frontRightWheelLinearSpeed;
  double rearRightWheelLinearSpeed;
  double frontRightWheelSteeringAngle;
};

std::ostream & operator<<(std::ostream & s, const OdometryFrame2FWS4WD & frame);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2FWS4WD_HPP_
