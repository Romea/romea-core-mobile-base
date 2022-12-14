#include "romea_core_mobile_base/info/MobileBaseInfo1FAS4WD.hpp"

namespace romea {

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo1FAS2RWD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " type:" << std::endl;
//  os << "  1FAS2RWD"<<std::endl;
//  os << " geometry:";
//  os << baseInformation.geometry<< std::endl;
//  os << " front axle steering control:" <<std::endl;
//  os << baseInformation.frontAxleSteeringControl<< std::endl;
//  os << " wheels speed control: " <<std::endl;
//  os << baseInformation.wheelsSpeedControl<< std::endl;
//  os << " intertia:" << std::endl;
//  os << baseInformation.inertia;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(const MobileBaseInfo1FAS4WD & baseInformation,
                             OneAxleSteeringKinematic::Parameters & kinematicParameters )
{
  const auto & geometry = baseInformation.geometry;
  const auto & wheelsSpeedCommand = baseInformation.wheelsSpeedControl.command;
  const auto & wheelsSpeedSensor = baseInformation.wheelsSpeedControl.sensor;
  const auto & axleSteeringCommand = baseInformation.frontAxleSteeringControl.command;
  const auto & axleSteeringSensor = baseInformation.frontAxleSteeringControl.sensor;
  const auto & controlPoint = baseInformation.controlPoint;

  kinematicParameters.frontWheelBase = geometry.axlesDistance/2. - controlPoint.x();
  kinematicParameters.rearWheelBase = geometry.axlesDistance/2.+ controlPoint.x();
  kinematicParameters.frontWheelTrack = geometry.frontAxle.wheelsDistance;
  kinematicParameters.rearWheelTrack = geometry.rearAxle.wheelsDistance;
  kinematicParameters.frontHubCarrierOffset = geometry.frontAxle.wheels.hubCarrierOffset;
  kinematicParameters.rearHubCarrierOffset = geometry.rearAxle.wheels.hubCarrierOffset;
  kinematicParameters.maximalSteeringAngle = axleSteeringCommand.maximalAngle;
  kinematicParameters.maximalSteeringAngularSpeed = axleSteeringCommand.maximalAngularSpeed;
  kinematicParameters.rearMaximalWheelLinearSpeed = wheelsSpeedCommand.maximalSpeed;
  kinematicParameters.maximalWheelLinearAcceleration = wheelsSpeedCommand.maximalAcceleration;
  kinematicParameters.wheelLinearSpeedVariance = std::pow(wheelsSpeedSensor.speedStd, 2.0);
  kinematicParameters.steeringAngleVariance = std::pow(axleSteeringSensor.angleStd, 2.0);
}

}  // namespace romea
