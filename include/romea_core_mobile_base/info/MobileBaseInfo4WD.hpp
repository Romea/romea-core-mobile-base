// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO4WD_HPP_
#define ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO4WD_HPP_

#include "romea_core_mobile_base/info/MobileBaseControl.hpp"
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"
#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/MecanumWheelSteeringKinematic.hpp"

namespace romea
{

struct MobileBaseInfo4WD
{
  TwoAxles<WheeledAxle, WheeledAxle> geometry;
  WheelSpeedControl wheelsSpeedControl;
  MobileBaseInertia inertia;
  Eigen::Vector3d controlPoint;
};

//std::ostream& operator<<(std::ostream& os, const MobileBaseInfo4WD & description);

void to_kinematic_parameters(
  const MobileBaseInfo4WD & baseInformation,
  SkidSteeringKinematic::Parameters & kinematicParameters);

void to_kinematic_parameters(
  const MobileBaseInfo4WD & baseInformation,
  MecanumWheelSteeringKinematic::Parameters & kinematicParameters);

}  // namespace romea

#endif // ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINFO4WD_HPP_
