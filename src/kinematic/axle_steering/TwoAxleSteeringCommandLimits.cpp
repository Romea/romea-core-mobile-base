//romea
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommandLimits.hpp"

//std
#include <limits>
#include <cassert>
#include <cmath>

namespace romea {

//--------------------------------------------------------------------------
TwoAxleSteeringCommandLimits::TwoAxleSteeringCommandLimits():
  longitudinalSpeed(),
  frontSteeringAngle(-M_PI_2,M_PI_2),
  rearSteeringAngle(-M_PI_2,M_PI_2)
{

}

//--------------------------------------------------------------------------
TwoAxleSteeringCommandLimits::TwoAxleSteeringCommandLimits(const double & minimalLongitudinalSpeed,
                                                           const double & maximalLongidudinalSpeed,
                                                           const double & maximalFrontSteeringAngle,
                                                           const double & maximalRearSteeringAngle):
  longitudinalSpeed(makeLongitudinalSpeedCommandLimits(minimalLongitudinalSpeed,
                                                       maximalLongidudinalSpeed)),
  frontSteeringAngle(makeSteeringAngleCommandLimits(maximalFrontSteeringAngle)),
  rearSteeringAngle(makeSteeringAngleCommandLimits(maximalRearSteeringAngle))
{

}

//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const TwoAxleSteeringCommandLimits & limits)
{
  os << "Command limits : " << std::endl;
  os << " longitudinal speed : [" <<limits.longitudinalSpeed.lower() << " "<< limits.longitudinalSpeed.upper()<<"]" << std::endl;
  os << " front stering angle : [" <<limits.frontSteeringAngle.lower() << " "<< limits.frontSteeringAngle.upper()<<"]";
  os << " rear stering angle : [" <<limits.rearSteeringAngle.lower() << " "<< limits.rearSteeringAngle.upper()<<"]";
  return os;
}


}//end romea

