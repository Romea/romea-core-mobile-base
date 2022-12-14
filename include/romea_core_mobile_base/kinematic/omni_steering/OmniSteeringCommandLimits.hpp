#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_OMNI_STEERING_OMNISTEERINGCOMMANDLIMITS_HPP_
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_OMNI_STEERING_OMNISTEERINGCOMMANDLIMITS_HPP_

#include "romea_core_mobile_base/kinematic/CommandLimits.hpp"

namespace romea {

struct OmniSteeringCommandLimits
{
  OmniSteeringCommandLimits();

  OmniSteeringCommandLimits(const double & minimalLongitudinalSpeed,
                            const double & maximalLongidudinalSpeed,
                            const double & maximalLateralSpeed,
                            const double & maximalAngularSpeed);

  Interval1D<double> longitudinalSpeed;
  Interval1D<double> lateralSpeed;
  Interval1D<double> angularSpeed;
};

std::ostream& operator<<(std::ostream& os, const OmniSteeringCommandLimits & limits);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC_OMNI_STEERING_OMNISTEERINGCOMMANDLIMITS_HPP_
