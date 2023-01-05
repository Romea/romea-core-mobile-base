// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2AS4WD_HPP_
#define ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2AS4WD_HPP_

// stl
#include <ostream>

namespace romea
{


struct OdometryFrame2AS4WD
{
  double frontAxleSteeringAngle;
  double frontLeftWheelLinearSpeed;
  double frontRightWheelLinearSpeed;

  double rearAxleSteeringAngle;
  double rearLeftWheelLinearSpeed;
  double rearRightWheelLinearSpeed;
};

std::ostream & operator<<(std::ostream & s, const OdometryFrame2AS4WD & frame);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2AS4WD_HPP_
