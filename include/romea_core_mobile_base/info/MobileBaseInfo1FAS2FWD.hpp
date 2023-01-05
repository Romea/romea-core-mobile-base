// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO1FAS2FWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO1FAS2FWD_HPP_

#include "romea_core_mobile_base/info/MobileBaseControl.hpp"
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"
#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"

namespace romea
{

struct MobileBaseInfo1FAS2FWD
{
  TwoAxles<WheeledAxle, WheeledAxle> geometry;
  SteeringAngleControl frontAxleSteeringControl;
  WheelSpeedControl frontWheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo1FAS2FWD & baseInformation);

void to_kinematic_parameters(
  const MobileBaseInfo1FAS2FWD & baseInformation,
  OneAxleSteeringKinematic::Parameters & kinematicParameters);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO1FAS2FWD_HPP_
