#ifndef romea_TwoAxleSteeringCommandLimits_hpp
#define romea_TwoAxleSteeringCommandLimits_hpp

#include "../CommandLimits.hpp"

namespace romea {


struct TwoAxleSteeringCommandLimits
{

  TwoAxleSteeringCommandLimits();

  TwoAxleSteeringCommandLimits(const double & minimalLongitudinalSpeed,
                               const double & maximalLongidudinalSpeed,
                               const double & maximalFrontSteeringAngle,
                               const double & maximalRearSteeringAngle);

  Interval1D<double> longitudinalSpeed;
  Interval1D<double> frontSteeringAngle;
  Interval1D<double> rearSteeringAngle;

};

std::ostream& operator<<(std::ostream& os, const TwoAxleSteeringCommandLimits & limits);

}//end romea
#endif
