#ifndef romea_OmniSteeringConstraints_hpp
#define romea_OmniSteeringConstraints_hpp

#include "../CommandLimits.hpp"

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


}//end romea
#endif
