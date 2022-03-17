#include "romea_core_mobile_base/info/MobileBaseInfo2FWS2RWD.hpp"

namespace romea {

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2FWS2RWD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " type:" << std::endl;
//  os << "  2FWS2RWD"<<std::endl;
//  os << " geometry:";
//  os << baseInformation.geometry<< std::endl;
//  os << " front wheels steering control:" <<std::endl;
//  os << baseInformation.frontWheelsSteeringControl<< std::endl;
//  os << " rear wheels speed control: " <<std::endl;
//  os << baseInformation.rearWheelsSpeedControl<< std::endl;
//  os << " intertia:" << std::endl;
//  os << baseInformation.inertia;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(const MobileBaseInfo2FWS2RWD & baseInformation,
                             TwoWheelSteeringKinematic::Parameters & kinematicParameters )
{
  const auto & geometry= baseInformation.geometry;
  const auto & wheelsSpeedCommand = baseInformation.rearWheelsSpeedControl.command;
  const auto & wheelsSpeedSensor = baseInformation.rearWheelsSpeedControl.sensor;
  const auto & wheelsSteeringCommand = baseInformation.frontWheelsSteeringControl.command;
  const auto & wheelsSteeringSensor = baseInformation.frontWheelsSteeringControl.sensor;
  const auto & controlPoint = baseInformation.controlPoint;

  kinematicParameters.frontWheelBase = geometry.axlesDistance/2. - controlPoint.x();
  kinematicParameters.rearWheelBase = geometry.axlesDistance/2.+ controlPoint.x();
  kinematicParameters.frontWheelTrack=geometry.frontAxle.wheelsDistance;
  kinematicParameters.rearWheelTrack=geometry.rearAxle.wheelsDistance;
  kinematicParameters.frontHubCarrierOffset = geometry.frontAxle.wheels.hubCarrierOffset;
  kinematicParameters.rearHubCarrierOffset = geometry.rearAxle.wheels.hubCarrierOffset;
  kinematicParameters.maximalWheelAngle = wheelsSteeringCommand.maximalAngle;
  kinematicParameters.maximalWheelAngularSpeed = wheelsSteeringCommand.maximalAngularSpeed;
  kinematicParameters.rearMaximalWheelSpeed = wheelsSpeedCommand.maximalSpeed;
  kinematicParameters.maximalWheelAcceleration = wheelsSpeedCommand.maximalAcceleration;
  kinematicParameters.wheelSpeedVariance = std::pow(wheelsSpeedSensor.speedStd,2.0);
  kinematicParameters.wheelAngleVariance = std::pow(wheelsSteeringSensor.angleStd,2.0);
}


////-----------------------------------------------------------------------------
//void to_kinematic_parameters(const MobileBaseInfo2FWS2RWD & baseInformation,
//                             OneAxleSteeringKinematic::Parameters & kinematicParameters )
//{
//  const auto & geometry= baseInformation.geometry;
//  const auto & wheelsSpeedCommand = baseInformation.rearWheelsSpeedControl.command;
//  const auto & wheelsSpeedSensor = baseInformation.rearWheelsSpeedControl.sensor;
//  const auto & wheelsSteeringCommand = baseInformation.frontWheelsSteeringControl.command;
//  const auto & wheelsSteeringSensor = baseInformation.frontWheelsSteeringControl.sensor;
//  const auto & controlPoint = baseInformation.controlPoint;

//  double maximalInstanenousCurvarture = TwoWheelSteeringKinematic::
//      computeMaximalInstantaneousCurvature(geometry.wheelbase,
//                                           geometry.frontAxle.wheelTrack/2.0,
//                                           wheelsSteeringCommand.maximalAngle);

//  double maximalSteeringAngle =TwoWheelSteeringKinematic::
//      computeSteeringAngle(maximalInstanenousCurvarture,geometry.wheelbase);


//  kinematicParameters.frontWheelBase = geometry.wheelbase/2. - controlPoint.x();
//  kinematicParameters.rearWheelBase = geometry.wheelbase/2.+ controlPoint.x();
//  kinematicParameters.frontWheelTrack=geometry.frontAxle.wheelTrack;
//  kinematicParameters.rearWheelTrack=geometry.rearAxle.wheelTrack;
//  kinematicParameters.frontHubCarrierOffset = geometry.frontAxle.wheels.hubCarrierOffset;
//  kinematicParameters.rearHubCarrierOffset = geometry.rearAxle.wheels.hubCarrierOffset;
//  kinematicParameters.maximalSteeringAngle = maximalSteeringAngle;
//#warning finir conversion MobileBaseInfo2FWS2RWD -> OneAxleSteeringKinematic::Parameters;
////  kinematicParameters.maximalSteeringAngularSpeed = wheelsSteeringCommand.maximalAngularSpeed;
//  kinematicParameters.rearMaximalWheelSpeed = wheelsSpeedCommand.maximalSpeed;
//  kinematicParameters.maximalWheelAcceleration = wheelsSpeedCommand.maximalAcceleration;
//  kinematicParameters.wheelSpeedVariance = std::pow(wheelsSpeedSensor.speedStd,2.0);
////  kinematicParameters.steeringAngleVariance = std::pow(wheelsSteeringSensor.angleStd,2.0);

//}

}
