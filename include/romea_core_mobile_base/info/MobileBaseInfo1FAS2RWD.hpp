#ifndef ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO1FAS2RWD_HPP
#define ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO1FAS2RWD_HPP

#include "romea_core_mobile_base/info/MobileBaseControl.hpp"
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"
#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"

namespace romea {

struct MobileBaseInfo1FAS2RWD
{
  TwoAxles<WheeledAxle, WheeledAxle> geometry;
  SteeringAngleControl frontAxleSteeringControl;
  WheelSpeedControl rearWheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo1FAS2RWD & baseInformation);

void to_kinematic_parameters(const MobileBaseInfo1FAS2RWD & baseInformation,
                             OneAxleSteeringKinematic::Parameters & kinematicParameters);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINFO1FAS2RWD_HPP
