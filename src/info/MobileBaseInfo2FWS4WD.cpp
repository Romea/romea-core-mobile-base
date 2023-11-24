// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "romea_core_mobile_base/info/MobileBaseInfo2FWS4WD.hpp"

namespace romea
{
namespace core
{

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2FWS4WD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " type:" << std::endl;
//  os << "  2FWS4WD"<<std::endl;
//  os << " geometry:";
//  os << baseInformation.geometry<< std::endl;
//  os << " front wheels steering control:" <<std::endl;
//  os << baseInformation.frontWheelsSteeringControl<< std::endl;
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
  const MobileBaseInfo2FWS4WD & baseInformation,
  TwoWheelSteeringKinematic::Parameters & kinematicParameters)
{
  const auto & geometry = baseInformation.geometry;
  const auto & wheelsSpeedCommand = baseInformation.wheelsSpeedControl.command;
  const auto & wheelsSpeedSensor = baseInformation.wheelsSpeedControl.sensor;
  const auto & wheelsSteeringCommand = baseInformation.frontWheelsSteeringControl.command;
  const auto & wheelsSteeringSensor = baseInformation.frontWheelsSteeringControl.sensor;
  const auto & controlPoint = baseInformation.controlPoint;

  kinematicParameters.frontWheelBase = geometry.axlesDistance / 2. - controlPoint.x();
  kinematicParameters.rearWheelBase = geometry.axlesDistance / 2. + controlPoint.x();
  kinematicParameters.frontWheelTrack = geometry.frontAxle.wheelsDistance;
  kinematicParameters.rearWheelTrack = geometry.rearAxle.wheelsDistance;
  kinematicParameters.frontHubCarrierOffset = geometry.frontAxle.wheels.hubCarrierOffset;
  kinematicParameters.rearHubCarrierOffset = geometry.rearAxle.wheels.hubCarrierOffset;
  kinematicParameters.maximalWheelSteeringAngle = wheelsSteeringCommand.maximalAngle;
  kinematicParameters.maximalWheelSteeringAngularSpeed = wheelsSteeringCommand.maximalAngularSpeed;
  kinematicParameters.rearMaximalWheelLinearSpeed = wheelsSpeedCommand.maximalSpeed;
  kinematicParameters.maximalWheelLinearAcceleration = wheelsSpeedCommand.maximalAcceleration;
  kinematicParameters.wheelLinearSpeedVariance = std::pow(wheelsSpeedSensor.speedStd, 2.0);
  kinematicParameters.wheelSteeringAngleVariance = std::pow(wheelsSteeringSensor.angleStd, 2.0);
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

}  // namespace core
}  // namespace romea
