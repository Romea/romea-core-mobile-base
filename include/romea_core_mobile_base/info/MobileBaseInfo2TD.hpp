#ifndef ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2TD_HPP_
#define ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2TD_HPP_

#include "romea_core_mobile_base/info/MobileBaseControl.hpp"
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"
#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo2TD
{
  ContinuousTrackedAxle geometry;
  WheelSpeedControl tracksSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2TD & base_information);

void to_kinematic_parameters(const MobileBaseInfo2TD & base_information,
                             SkidSteeringKinematic::Parameters & kinematic_parameters);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2TD_HPP_
