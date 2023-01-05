// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME1FAS2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME1FAS2RWD_HPP_

// stl
#include <ostream>

namespace romea
{

struct OdometryFrame1FAS2RWD
{
  double frontAxleSteeringAngle;
  double rearLeftWheelLinearSpeed;
  double rearRightWheelLinearSpeed;
};

std::ostream & operator<<(std::ostream & s, const OdometryFrame1FAS2RWD & frame);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__ODOMETRY__ODOMETRYFRAME1FAS2RWD_HPP_
