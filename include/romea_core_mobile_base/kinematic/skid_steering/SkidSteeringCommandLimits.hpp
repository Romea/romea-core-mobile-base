#ifndef romea_SkidSteeringCommandLimits_hpp
#define romea_SkidSteeringCommandLimits_hpp

#include "../CommandLimits.hpp"

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

}//end romea
#endif
