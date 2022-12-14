#ifndef ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINERTIA_HPP_
#define ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINERTIA_HPP_

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

#endif  // ROMEA_CORE_MOBILE_BASE_INFO_MOBILEBASEINERTIA_HPP_
