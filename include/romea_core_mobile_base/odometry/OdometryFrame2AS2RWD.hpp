// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2AS2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2AS2RWD_HPP_

// stl
#include <ostream>

namespace romea
{


struct OdometryFrame2AS2RWD
{
  double frontAxleSteeringAngle;
  double rearAxleSteeringAngle;
  double rearLeftWheelLinearSpeed;
  double rearRightWheelLinearSpeed;

};

std::ostream & operator<<(std::ostream & s, const OdometryFrame2AS2RWD & frame);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2AS2RWD_HPP_
