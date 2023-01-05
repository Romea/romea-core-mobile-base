// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2TD_HPP_
#define ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2TD_HPP_

// stl
#include <ostream>

namespace romea
{

struct OdometryFrame2TD
{
  double leftTrackLinearSpeed;
  double rightTrackLinearSpeed;
};

std::ostream & operator<<(std::ostream & s, const OdometryFrame2TD & frame);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME2TD_HPP_
