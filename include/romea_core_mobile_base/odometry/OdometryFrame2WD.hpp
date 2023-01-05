// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2WD_HPP_
#define ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2WD_HPP_

// stl
#include <ostream>

namespace romea
{

struct OdometryFrame2WD
{
  double leftWheelLinearSpeed;
  double rightWheelLinearSpeed;
};

std::ostream & operator<<(std::ostream & s, const OdometryFrame2WD & frame);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2WD_HPP_
