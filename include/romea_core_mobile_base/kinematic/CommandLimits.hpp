#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_COMMANDLIMITS_HPP_
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_COMMANDLIMITS_HPP_

#include <romea_core_common/math/Interval.hpp>

namespace romea {

Interval1D<double> makeSymmetricCommandLimits(const double & maximalAbsoluteCommand);

Interval1D<double> makeSteeringAngleCommandLimits(const double & maximalAbsoluteSteeringAngle);

Interval1D<double> makeLongitudinalSpeedCommandLimits(const double & maximalBackwardSpeed,
                                                      const double & maximalForwardSpeed);

double clamp(const double & value, const Interval1D<double> & limits);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC_COMMANDLIMITS_HPP_
