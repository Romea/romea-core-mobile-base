// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINERTIA_HPP_
#define ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINERTIA_HPP_

#include <Eigen/Core>

namespace romea
{

struct MobileBaseInertia
{
  double mass;
  Eigen::Vector3d center;
  double zMoment;
};

//  std::ostream& operator<<(std::ostream& os, const MobileBaseInertia & inertia);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__INFO__MOBILEBASEINERTIA_HPP_
