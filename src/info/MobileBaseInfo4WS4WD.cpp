#include "romea_core_mobile_base/info/MobileBaseInfo4WS4WD.hpp"
#include <romea_core_common/math/Algorithm.hpp>

namespace romea {

//-----------------------------------------------------------------------------
MobileBaseInfo4WS4WD::MobileBaseInfo4WS4WD():
  geometry(),
  wheelsSteeringControl(),
  wheelsSpeedControl(),
  controlPoint(Eigen::Vector3d::Zero())
{

}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const MobileBaseInfo4WS4WD & baseInformation)
{
  os << "Base information:" << std::endl;
  os << " type:" << std::endl;
  os << "  4WS4WD"<<std::endl;
  os << " geometry:";
  os << baseInformation.geometry<< std::endl;
  os << " wheels steering control:" <<std::endl;
  os << baseInformation.wheelsSteeringControl<< std::endl;
  os << " wheels speed control: " <<std::endl;
  os << baseInformation.wheelsSpeedControl<< std::endl;
  os << " intertia:" << std::endl;
  os << baseInformation.inertia;
  os << " control point:" << std::endl;
  os << "  " << baseInformation.controlPoint << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(const MobileBaseInfo4WS4WD & baseInformation,
                             FourWheelSteeringKinematic::Parameters & kinematicParameters )
{
  const auto & geometry= baseInformation.geometry;
  const auto & wheelsSpeedCommand = baseInformation.wheelsSpeedControl.command;
  const auto & wheelsSpeedSensor = baseInformation.wheelsSpeedControl.sensor;
  const auto & wheelsSteeringCommand = baseInformation.wheelsSteeringControl.command;
  const auto & wheelsSteeringSensor = baseInformation.wheelsSteeringControl.sensor;
  const auto & controlPoint = baseInformation.controlPoint;

  if(near(geometry.frontAxle.wheelTrack,
          geometry.rearAxle.wheelTrack))
  {
    std::stringstream ss;
    ss << "Unable to convert base information to four wheel steering kinematic";
    ss << "because wheel track of front and rear axles are not equals";
    throw std::runtime_error(ss.str());
  }

  if(near(geometry.rearAxle.wheels.hubCarrierOffset,
          geometry.rearAxle.wheels.hubCarrierOffset))
  {
    std::stringstream ss;
    ss << "Unable to convert base information to four wheel steering kinematic";
    ss << "because wheel hub carrier offset of front and rear axles are not equals";
    throw std::runtime_error(ss.str());
  }

  kinematicParameters.frontWheelBase = geometry.wheelbase/2. - controlPoint.x();
  kinematicParameters.rearWheelBase = geometry.wheelbase/2.+ controlPoint.x();
  kinematicParameters.wheelTrack=geometry.rearAxle.wheelTrack;
  kinematicParameters.hubCarrierOffset = geometry.rearAxle.wheels.hubCarrierOffset;
  kinematicParameters.maximalWheelAngle = wheelsSteeringCommand.maximalAngle;
  kinematicParameters.maximalWheelAngularSpeed = wheelsSteeringCommand.maximalAngularSpeed;
  kinematicParameters.maximalWheelSpeed = wheelsSpeedCommand.maximalSpeed;
  kinematicParameters.maximalWheelAcceleration = wheelsSpeedCommand.maximalAcceleration;
  kinematicParameters.wheelSpeedVariance = std::pow(wheelsSpeedSensor.speedStd,2.0);
  kinematicParameters.wheelAngleVariance = std::pow(wheelsSteeringSensor.angleStd,2.0);
}

}
