#include "romea_core_mobile_base/info/MobileBaseInfo4WS4WD.hpp"
#include <romea_core_common/math/Algorithm.hpp>

namespace romea {

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo4WS4WD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " type:" << std::endl;
//  os << "  4WS4WD"<<std::endl;
//  os << " geometry:";
//  os << baseInformation.geometry<< std::endl;
//  os << " wheels steering control:" <<std::endl;
//  os << baseInformation.wheelsSteeringControl<< std::endl;
//  os << " wheels speed control: " <<std::endl;
//  os << baseInformation.wheelsSpeedControl<< std::endl;
//  os << " intertia:" << std::endl;
//  os << baseInformation.inertia;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(const MobileBaseInfo4WS4WD & baseInformation,
                             FourWheelSteeringKinematic::Parameters & kinematicParameters )
{
  const auto & geometry = baseInformation.geometry;
  const auto & wheelsSpeedCommand = baseInformation.wheelsSpeedControl.command;
  const auto & wheelsSpeedSensor = baseInformation.wheelsSpeedControl.sensor;
  const auto & wheelsSteeringCommand = baseInformation.wheelsSteeringControl.command;
  const auto & wheelsSteeringSensor = baseInformation.wheelsSteeringControl.sensor;
  const auto & controlPoint = baseInformation.controlPoint;

  if (!near(geometry.frontAxle.wheelsDistance, geometry.rearAxle.wheelsDistance))
  {
    std::stringstream ss;
    ss << "Unable to convert base information to four wheel steering kinematic";
    ss << "because distance between wheels of front and rear axles are not equals";
    throw std::runtime_error(ss.str());
  }

  if (!near(geometry.frontAxle.wheels.hubCarrierOffset, geometry.rearAxle.wheels.hubCarrierOffset))
  {
    std::stringstream ss;
    ss << "Unable to convert base information to four wheel steering kinematic";
    ss << "because wheel hub carrier offset of front and rear axles are not equals";
    throw std::runtime_error(ss.str());
  }

  kinematicParameters.frontWheelBase = geometry.axlesDistance/2. - controlPoint.x();
  kinematicParameters.rearWheelBase = geometry.axlesDistance/2.+ controlPoint.x();
  kinematicParameters.wheelTrack = geometry.rearAxle.wheelsDistance;
  kinematicParameters.hubCarrierOffset = geometry.rearAxle.wheels.hubCarrierOffset;
  kinematicParameters.maximalWheelSteeringAngle = wheelsSteeringCommand.maximalAngle;
  kinematicParameters.maximalWheelSteeringAngularSpeed = wheelsSteeringCommand.maximalAngularSpeed;
  kinematicParameters.maximalWheelLinearSpeed = wheelsSpeedCommand.maximalSpeed;
  kinematicParameters.maximalWheelLinearAcceleration = wheelsSpeedCommand.maximalAcceleration;
  kinematicParameters.wheelLinearSpeedVariance = std::pow(wheelsSpeedSensor.speedStd, 2.0);
  kinematicParameters.wheelSteeringAngleVariance = std::pow(wheelsSteeringSensor.angleStd, 2.0);
}

}  // namespace romea
