#include "romea_core_mobile_base/info/MobileBaseInfo1FWS2RWD.hpp"

namespace romea {

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo1FWS2RWD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " type:" << std::endl;
//  os << "  1FWS2RWD"<<std::endl;
//  os << " geometry:";
//  os << baseInformation.geometry<< std::endl;
//  os << " front wheel steering control:" <<std::endl;
//  os << baseInformation.frontWheelSteeringControl<< std::endl;
//  os << " rear wheels speed control: " <<std::endl;
//  os << baseInformation.rearWheelsSpeedControl<< std::endl;
//  os << " intertia:" << std::endl;
//  os << baseInformation.inertia;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(const MobileBaseInfo1FWS2RWD & baseInformation,
                             OneAxleSteeringKinematic::Parameters & kinematicParameters )
{
  const auto & geometry= baseInformation.geometry;
  const auto & wheelsSpeedCommand = baseInformation.rearWheelsSpeedControl.command;
  const auto & wheelsSpeedSensor = baseInformation.rearWheelsSpeedControl.sensor;
  const auto & wheelSteeringCommand = baseInformation.frontWheelSteeringControl.command;
  const auto & wheelSteeringSensor = baseInformation.frontWheelSteeringControl.sensor;
  const auto & controlPoint = baseInformation.controlPoint;

  kinematicParameters.frontWheelBase = geometry.axlesDistance/2. - controlPoint.x();
  kinematicParameters.rearWheelBase = geometry.axlesDistance/2.+ controlPoint.x();
  kinematicParameters.frontWheelTrack=geometry.frontAxle.wheelsDistance;
  kinematicParameters.rearWheelTrack=geometry.rearAxle.wheelsDistance;
  kinematicParameters.frontHubCarrierOffset = geometry.frontAxle.wheels.hubCarrierOffset;
  kinematicParameters.rearHubCarrierOffset = geometry.rearAxle.wheels.hubCarrierOffset;
  kinematicParameters.maximalSteeringAngle = wheelSteeringCommand.maximalAngle;
  kinematicParameters.maximalSteeringAngularSpeed = wheelSteeringCommand.maximalAngularSpeed;
  kinematicParameters.rearMaximalWheelSpeed = wheelsSpeedCommand.maximalSpeed;
  kinematicParameters.maximalWheelAcceleration = wheelsSpeedCommand.maximalAcceleration;
  kinematicParameters.wheelSpeedVariance = std::pow(wheelsSpeedSensor.speedStd,2.0);
  kinematicParameters.steeringAngleVariance = std::pow(wheelSteeringSensor.angleStd,2.0);
}

}
