// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <ostream>

// romea
#include "romea_core_mobile_base/odometry/OdometryFrame2FWS2FWD.hpp"

namespace romea
{

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2FWS2FWD & frame)
{
  s << "frontLeftWheelLinearSpeed : " << frame.frontLeftWheelLinearSpeed << std::endl;
  s << "frontLeftWheelSteeringAngle : " << frame.frontLeftWheelSteeringAngle << std::endl;
  s << "frontRightWheelLinearSpeed : " << frame.frontRightWheelLinearSpeed << std::endl;
  s << "frontRightWheelSteeringAngle : " << frame.frontRightWheelSteeringAngle << std::endl;
  return s;
}


}  // namespace romea
