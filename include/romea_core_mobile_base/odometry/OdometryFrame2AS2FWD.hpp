// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2AS2FWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2AS2FWD_HPP_

// stl
#include <ostream>

namespace romea
{


struct OdometryFrame2AS2FWD
{
  double frontAxleSteeringAngle;
  double rearAxleSteeringAngle;

  double frontLeftWheelLinearSpeed;
  double frontRightWheelLinearSpeed;
};

std::ostream & operator<<(std::ostream & s, const OdometryFrame2AS2FWD & frame);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2AS2FWD_HPP_
