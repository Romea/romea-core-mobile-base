#ifndef _romea_MobileBaseInertia_hpp_
#define _romea_MobileBaseInertia_hpp_

#include <Eigen/Core>

namespace romea
{

  struct MobileBaseInertia
  {
    MobileBaseInertia();
    double mass;
    Eigen::Vector3d center;
    double zInertialMoment;
  };

  std::ostream& operator<<(std::ostream& os, const MobileBaseInertia & inertia);

}

#endif
