#include "romea_core_mobile_base/info/MobileBaseInfo2FWS2FWD.hpp"

namespace romea {

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2FWS2FWD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " type:" << std::endl;
//  os << "  2FWS2FWD"<<std::endl;
//  os << " geometry:";
//  os << baseInformation.geometry<< std::endl;
//  os << " front wheels steering control:" <<std::endl;
//  os << baseInformation.frontWheelsSteeringControl<< std::endl;
//  os << " front wheels speed control: " <<std::endl;
//  os << baseInformation.frontWheelsSpeedControl<< std::endl;
//  os << " intertia:" << std::endl;
//  os << baseInformation.inertia;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(const MobileBaseInfo2FWS2FWD & baseInformation,
                             TwoWheelSteeringKinematic::Parameters & kinematicParameters )
{
  const auto & geometry= baseInformation.geometry;
  const auto & wheelsSpeedCommand = baseInformation.frontWheelsSpeedControl.command;
  const auto & wheelsSpeedSensor = baseInformation.frontWheelsSpeedControl.sensor;
  const auto & wheelsSteeringCommand = baseInformation.frontWheelsSteeringControl.command;
  const auto & wheelsSteeringSensor = baseInformation.frontWheelsSteeringControl.sensor;
  const auto & controlPoint = baseInformation.controlPoint;

  kinematicParameters.frontWheelBase = geometry.axlesDistance/2. - controlPoint.x();
  kinematicParameters.rearWheelBase = geometry.axlesDistance/2.+ controlPoint.x();
  kinematicParameters.frontWheelTrack = geometry.frontAxle.wheelsDistance;
  kinematicParameters.rearWheelTrack = geometry.rearAxle.wheelsDistance;
  kinematicParameters.frontHubCarrierOffset = geometry.frontAxle.wheels.hubCarrierOffset;
  kinematicParameters.rearHubCarrierOffset = geometry.rearAxle.wheels.hubCarrierOffset;
  kinematicParameters.maximalWheelSteeringAngle = wheelsSteeringCommand.maximalAngle;
  kinematicParameters.maximalWheelSteeringAngularSpeed = wheelsSteeringCommand.maximalAngularSpeed;
  kinematicParameters.frontMaximalWheelLinearSpeed = wheelsSpeedCommand.maximalSpeed;
  kinematicParameters.maximalWheelLinearAcceleration = wheelsSpeedCommand.maximalAcceleration;
  kinematicParameters.wheelLinearSpeedVariance = std::pow(wheelsSpeedSensor.speedStd,2.0);
  kinematicParameters.wheelSteeringAngleVariance = std::pow(wheelsSteeringSensor.angleStd,2.0);
}

}
