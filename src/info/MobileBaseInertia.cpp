#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"


namespace romea
{

  MobileBaseInertia::MobileBaseInertia():
    mass(std::numeric_limits<double>::quiet_NaN()),
    center(Eigen::Vector3d::Zero()),
    zMoment(std::numeric_limits<double>::quiet_NaN())
  {

  }


//  std::ostream& operator<<(std::ostream& os, const MobileBaseInertia & inertia);

}
