#ifndef ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2FWS2FWD_HPP_
#define ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2FWS2FWD_HPP_

#include "romea_core_mobile_base/info/MobileBaseControl.hpp"
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"
#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo2FWS2FWD
{
  TwoAxles<WheeledAxle, WheeledAxle> geometry;
  SteeringAngleControl frontWheelsSteeringControl;
  WheelSpeedControl frontWheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2FWS2FWD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo2FWS2FWD & baseInformation,
                             TwoWheelSteeringKinematic::Parameters & kinematicParameters);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2FWS2FWD_HPP_
