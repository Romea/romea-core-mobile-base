//romea
#include "romea_core_mobile_base/info/MobileBaseInfo4WD.hpp"
#include <romea_core_common/math/Algorithm.hpp>

//std
#include <cmath>
#include <sstream>

namespace romea {

////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo4WD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " geometry:" << std::endl;
//  os << baseInformation.geometry << std::endl;
//  os << " wheels speed control:" << std::endl;
//  os << baseInformation.wheelsSpeedControl<< std::endl;
//  os << " inertia:" << std::endl;
//  os << baseInformation.inertia<< std::endl;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(const MobileBaseInfo4WD & baseInformation,
                             SkidSteeringKinematic::Parameters & kinematicParameters )
{
  const auto & geometry = baseInformation.geometry;
  const auto & wheelsCommand = baseInformation.wheelsSpeedControl.command;
  const auto & wheelsSensor = baseInformation.wheelsSpeedControl.sensor;

  if(near(geometry.frontAxle.wheelsDistance,geometry.rearAxle.wheelsDistance))
  {
    std::stringstream ss;
    ss << "Unable to convert base information to skid steering kinematic";
    ss << "because distance between wheels of front and rear axles are not equals";
    throw std::runtime_error(ss.str());
  }

  kinematicParameters.wheelTrack = geometry.frontAxle.wheelsDistance;
  kinematicParameters.maximalWheelSpeed = wheelsCommand.maximalSpeed;
  kinematicParameters.wheelSpeedVariance = std::pow(wheelsSensor.speedStd,2);
  kinematicParameters.maximalWheelAcceleration = wheelsCommand.maximalAcceleration;
}

}
