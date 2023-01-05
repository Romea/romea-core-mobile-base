// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include "romea_core_mobile_base/info/MobileBaseInfo2WD.hpp"

// std
#include <cmath>

namespace romea
{

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2WD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " geometry:" << std::endl;
//  os << baseInformation.geometry << std::endl;
//  os << " tracks speed control:" << std::endl;
//  os << baseInformation.wheelsSpeedControl<< std::endl;
//  os << " inertia:" << std::endl;
//  os << baseInformation.inertia<< std::endl;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(
  const MobileBaseInfo2WD & baseInformation,
  SkidSteeringKinematic::Parameters & kinematicParameters)
{
  const auto & geometry = baseInformation.geometry;
  const auto & wheelsCommand = baseInformation.wheelsSpeedControl.command;
  const auto & wheelsSensor = baseInformation.wheelsSpeedControl.sensor;

  kinematicParameters.wheelTrack = geometry.wheelsDistance;
  kinematicParameters.maximalWheelLinearSpeed = wheelsCommand.maximalSpeed;
  kinematicParameters.wheelLinearSpeedVariance = std::pow(wheelsSensor.speedStd, 2);
  kinematicParameters.maximalWheelLinearAcceleration = wheelsCommand.maximalAcceleration;
}

}
