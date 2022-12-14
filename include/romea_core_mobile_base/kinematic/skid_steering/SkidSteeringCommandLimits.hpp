#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_SKIDSTEERINGCOMMANDLIMITS_HPP_
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_SKIDSTEERINGCOMMANDLIMITS_HPP_

#include "romea_core_mobile_base/kinematic/CommandLimits.hpp"

namespace romea {

struct SkidSteeringCommandLimits
{
  SkidSteeringCommandLimits();

  SkidSteeringCommandLimits(const double & minimalLongitudinalSpeed,
                            const double & maximalLongidudinalSpeed,
                            const double & maximalAngularSpeed);

  Interval1D<double> longitudinalSpeed;
  Interval1D<double> angularSpeed;
};

std::ostream& operator<<(std::ostream& os, const SkidSteeringCommandLimits & limits);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_SKIDSTEERINGCOMMANDLIMITS_HPP_
