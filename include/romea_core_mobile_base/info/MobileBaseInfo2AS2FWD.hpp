// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO2AS2FWD_HPP_
#define ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO2AS2FWD_HPP_

#include "romea_core_mobile_base/info/MobileBaseControl.hpp"
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"
#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"

namespace romea
{

struct MobileBaseInfo2AS2FWD
{
  TwoAxles<WheeledAxle, WheeledAxle> geometry;
  SteeringAngleControl axlesSteeringControl;
  WheelSpeedControl frontWheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo2AS4WD & baseInformation);

void to_kinematic_parameters(
  const MobileBaseInfo2AS2FWD & baseInformation,
  TwoAxleSteeringKinematic::Parameters & kinematicParameters);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO2AS2FWD_HPP_
