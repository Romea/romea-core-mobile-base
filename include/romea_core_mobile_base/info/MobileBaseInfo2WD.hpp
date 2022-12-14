#ifndef ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2WD_HPP_
#define ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2WD_HPP_

#include "romea_core_mobile_base/info/MobileBaseControl.hpp"
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"
#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo2WD
{
  WheeledAxle geometry;
  WheelSpeedControl wheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2WD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo2WD & baseInformation,
                             SkidSteeringKinematic::Parameters & kinematicParameters);

}  // namespace romea

#endif // ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2WD_HPP_
