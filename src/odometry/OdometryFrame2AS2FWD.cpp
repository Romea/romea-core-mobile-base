// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <ostream>

// romea
#include "romea_core_mobile_base/odometry/OdometryFrame2AS2FWD.hpp"

namespace romea
{

//--------------------------------------------------------------------------
std::ostream & operator<<(std::ostream & s, const OdometryFrame2AS2FWD & frame)
{
  s << "frontAxleSteeringAngle : " << frame.frontAxleSteeringAngle << std::endl;
  s << "rearAxleSteeringAngle : " << frame.rearAxleSteeringAngle << std::endl;
  s << "frontLeftWheelLinearSpeed : " << frame.frontLeftWheelLinearSpeed << std::endl;
  s << "frontRightWheelLinearSpeed : " << frame.frontRightWheelLinearSpeed << std::endl;
  return s;
}

}  //  namespace romea
