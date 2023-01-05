// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <ostream>

// romea
#include "romea_core_mobile_base/odometry/OdometryFrame4WD.hpp"

namespace romea
{

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame4WD & frame)
{
  s << "frontLeftWheelLinearSpeed : " << frame.frontLeftWheelLinearSpeed << std::endl;
  s << "frontRightWheelLinearSpeed : " << frame.frontRightWheelLinearSpeed << std::endl;
  s << "rearLeftWheelLinearSpeed : " << frame.rearLeftWheelLinearSpeed << std::endl;
  s << "rearRightWheelLinearSpeed : " << frame.rearRightWheelLinearSpeed << std::endl;
  return s;
}

}  // namespace romea
