//romea
#include "romea_core_mobile_base/info/MobileBaseInfo2THD.hpp"

//std
#include <cmath>

namespace romea {

////-----------------------------------------------------------------------------
//MobileBaseInfo2TD::MobileBaseInfo2TD():
//  geometry(),
//  tracksSpeedControl(),
//  inertia(),
//  controlPoint(Eigen::Vector3d::Zero())
//{
//};


////-----------------------------------------------------------------------------
//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2TD & baseInformation)
//{
//  os << "Base information:" << std::endl;
//  os << " type:"<< std::endl;
//  os << "  2TD" << std::endl;
//  os << " geometry:" << std::endl;
//  os << baseInformation.geometry << std::endl;
//  os << " tracks speed control:" << std::endl;
//  os << baseInformation.tracksSpeedControl<< std::endl;
//  os << " inertia:" << std::endl;
//  os << baseInformation.inertia<< std::endl;
//  os << " control point:" << std::endl;
//  os << "  " << baseInformation.controlPoint << std::endl;
//  return os;
//}

//-----------------------------------------------------------------------------
void to_kinematic_parameters(const MobileBaseInfo2THD &base_information,
                             SkidSteeringKinematic::Parameters & kinematic_parameters )
{
 const auto & geometry = base_information.geometry;
 const auto & tracksCommand = base_information.tracksSpeedControl.command;
 const auto & tracksSensor = base_information.tracksSpeedControl.sensor;

 kinematic_parameters.wheelTrack = geometry.tracksDistance;
 kinematic_parameters.maximalWheelSpeed = tracksCommand.maximalSpeed;
 kinematic_parameters.wheelSpeedVariance = std::pow(tracksSensor.speedStd,2);
 kinematic_parameters.maximalWheelAcceleration = tracksCommand.maximalAcceleration;
}

}