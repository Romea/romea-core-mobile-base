// std
#include <cmath>

// romea
#include "romea_core_mobile_base/kinematic/CommandLimits.hpp"
#include <romea_core_common/math/Algorithm.hpp>


namespace romea {

//-----------------------------------------------------------------------------
Interval1D<double> makeSymmetricCommandLimits(const double & maximalAbsoluteCommand)
{
   return Interval1D<double>(-maximalAbsoluteCommand, maximalAbsoluteCommand);
}

//-----------------------------------------------------------------------------
Interval1D<double> makeSteeringAngleCommandLimits(const double & maximalAbsoluteSteeringAngle)
{
  if (maximalAbsoluteSteeringAngle < 0 || maximalAbsoluteSteeringAngle > M_PI_2)
  {
    std::stringstream ss;
    ss << " Steering angle limits:";
    ss << " Unable to set upper angle to ";
    ss << maximalAbsoluteSteeringAngle ;
    ss << " because this value is not in range [0 pi/2]";
    throw std::runtime_error(ss.str());
  }

  return Interval1D<double>(-maximalAbsoluteSteeringAngle, maximalAbsoluteSteeringAngle);
}


//-----------------------------------------------------------------------------
Interval1D<double> makeLongitudinalSpeedCommandLimits(const double & maximalBackwardSpeed,
                                                      const double & maximalForwardSpeed)
{
  if (maximalBackwardSpeed > 0)
  {
    std::stringstream ss;
    ss << " Longitudinal speed limits : ";
    ss << " Unable to set lower speed to ";
    ss << maximalBackwardSpeed ;
    ss << " because its a positive value";
    throw std::runtime_error(ss.str());
  }

  if (maximalForwardSpeed < 0)
  {
    std::stringstream ss;
    ss << " Longitudinal speed limits : ";
    ss << " Unable to set upper speed to ";
    ss << maximalForwardSpeed ;
    ss << " because it's a negative value";
    throw std::runtime_error(ss.str());
  }

  return Interval1D<double>(maximalBackwardSpeed, maximalForwardSpeed);
}

//-----------------------------------------------------------------------------
double clamp(const double & value, const Interval1D<double> &limits)
{
  return clamp(value, limits.lower(), limits.upper());
}

}  // namespace romea

