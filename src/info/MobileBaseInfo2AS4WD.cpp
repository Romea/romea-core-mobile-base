// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_mobile_base/info/MobileBaseInfo2AS4WD.hpp"

namespace romea
{

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2AS4WD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " type:" << std::endl;
//  os << "  2AS4WD"<<std::endl;
//  os << " geometry:";
//  os << baseInformation.geometry<< std::endl;
//  os << " axles steering control:" <<std::endl;
//  os << baseInformation.axlesSteeringControl<< std::endl;
//  os << " wheels speed control: " <<std::endl;
//  os << baseInformation.wheelsSpeedControl<< std::endl;
//  os << " intertia:" << std::endl;
//  os << baseInformation.inertia;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(
  const MobileBaseInfo2AS4WD & baseInformation,
  TwoAxleSteeringKinematic::Parameters & kinematicParameters)
{
  const auto & geometry = baseInformation.geometry;
  const auto & wheelsSpeedCommand = baseInformation.wheelsSpeedControl.command;
  const auto & wheelsSpeedSensor = baseInformation.wheelsSpeedControl.sensor;
  const auto & axlesSteeringCommand = baseInformation.axlesSteeringControl.command;
  const auto & axlesSteeringSensor = baseInformation.axlesSteeringControl.sensor;
  const auto & controlPoint = baseInformation.controlPoint;

  kinematicParameters.frontWheelBase = geometry.axlesDistance / 2. - controlPoint.x();
  kinematicParameters.rearWheelBase = geometry.axlesDistance / 2. + controlPoint.x();
  kinematicParameters.frontWheelTrack = geometry.frontAxle.wheelsDistance;
  kinematicParameters.rearWheelTrack = geometry.rearAxle.wheelsDistance;
  kinematicParameters.frontHubCarrierOffset = geometry.frontAxle.wheels.hubCarrierOffset;
  kinematicParameters.rearHubCarrierOffset = geometry.rearAxle.wheels.hubCarrierOffset;
  kinematicParameters.frontMaximalSteeringAngle = axlesSteeringCommand.maximalAngle;
  kinematicParameters.rearMaximalSteeringAngle = axlesSteeringCommand.maximalAngle;
  kinematicParameters.maximalSteeringAngularSpeed = axlesSteeringCommand.maximalAngularSpeed;
  kinematicParameters.frontMaximalWheelLinearSpeed = wheelsSpeedCommand.maximalSpeed;
  kinematicParameters.rearMaximalWheelLinearSpeed = wheelsSpeedCommand.maximalSpeed;
  kinematicParameters.maximalWheelLinearAcceleration = wheelsSpeedCommand.maximalAcceleration;
  kinematicParameters.wheelLinearSpeedVariance = std::pow(wheelsSpeedSensor.speedStd, 2.0);
  kinematicParameters.steeringAngleVariance = std::pow(axlesSteeringSensor.angleStd, 2.0);
}

}  // namespace romea
