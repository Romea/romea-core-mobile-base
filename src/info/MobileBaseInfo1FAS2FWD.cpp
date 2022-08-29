#include "romea_core_mobile_base/info/MobileBaseInfo1FAS2FWD.hpp"

namespace romea {

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo1FAS2FWD & baseInformation)
//{
//  os << "Base information:"<< std::endl;
//  os << " type:"<< std::endl;
//  os << "  1FAS2FWD"<< std::endl;
//  os << " geometry:"<< std::endl;
//  os << baseInformation.geometry<< std::endl;
//  os << " front axle steering control:" <<std::endl;
//  os << baseInformation.frontAxleSteeringControl<< std::endl;
//  os << " frontwheels speed control: " <<std::endl;
//  os << baseInformation.frontWheelsSpeedControl<< std::endl;
//  os << " intertia:" << std::endl;
//  os << baseInformation.inertia;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(const MobileBaseInfo1FAS2FWD & baseInformation,
                             OneAxleSteeringKinematic::Parameters & kinematicParameters )
{
  const auto & geometry= baseInformation.geometry;
  const auto & wheelsSpeedCommand = baseInformation.frontWheelsSpeedControl.command;
  const auto & wheelsSpeedSensor = baseInformation.frontWheelsSpeedControl.sensor;
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
  kinematicParameters.frontMaximalWheelLinearSpeed = wheelsSpeedCommand.maximalSpeed;
  kinematicParameters.maximalWheelLinearAcceleration = wheelsSpeedCommand.maximalAcceleration;
  kinematicParameters.wheelLinearSpeedVariance = std::pow(wheelsSpeedSensor.speedStd,2.0);
  kinematicParameters.steeringAngleVariance = std::pow(axleSteeringSensor.angleStd,2.0);
}

}
