#ifndef ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2AS2RWD_HPP_
#define ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2AS2RWD_HPP_

#include "romea_core_mobile_base/info/MobileBaseControl.hpp"
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"
#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo2AS2RWD
{
  TwoAxles<WheeledAxle, WheeledAxle> geometry;
  SteeringAngleControl axlesSteeringControl;
  WheelSpeedControl rearWheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2AS4WD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo2AS2RWD & baseInformation,
                             TwoAxleSteeringKinematic::Parameters & kinematicParameters);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO2AS2RWD_HPP_
