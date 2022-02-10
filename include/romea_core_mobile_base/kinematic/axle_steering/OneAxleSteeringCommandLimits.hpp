#ifndef romea_OneAxleSteeringCommandLimits_hpp
#define romea_OneAxleSteeringCommandLimits_hpp

#include "../CommandLimits.hpp"

namespace romea {


struct OneAxleSteeringCommandLimits
{
  OneAxleSteeringCommandLimits();

  OneAxleSteeringCommandLimits(const double & minimalLongitudinalSpeed,
                               const double & maximalLongidudinalSpeed,
                               const double & maximalSteeringAngle);

  Interval1D<double> longitudinalSpeed;
  Interval1D<double> steeringAngle;
};

std::ostream& operator<<(std::ostream& os, const OneAxleSteeringCommandLimits & limits);


}//end romea
#endif
