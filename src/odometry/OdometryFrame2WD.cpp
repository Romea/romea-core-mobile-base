// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <ostream>

// romea
#include "romea_core_mobile_base/odometry/OdometryFrame2WD.hpp"

namespace romea
{

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2WD & frame)
{
  s << "leftWheelLinearSpeed : " << frame.leftWheelLinearSpeed << std::endl;
  s << "rightWheelLinearSpeed : " << frame.rightWheelLinearSpeed << std::endl;
  return s;
}

}  // namespace romea
