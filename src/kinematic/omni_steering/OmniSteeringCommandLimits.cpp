//romea
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommandLimits.hpp"

//std
#include <limits>
#include <cassert>

namespace romea {


//--------------------------------------------------------------------------
OmniSteeringCommandLimits::OmniSteeringCommandLimits():
  longitudinalSpeed(),
  lateralSpeed(),
  angularSpeed()
{

}

//--------------------------------------------------------------------------
OmniSteeringCommandLimits::OmniSteeringCommandLimits(const double & minimalLongitudinalSpeed,
                                                     const double & maximalLongidudinalSpeed,
                                                     const double & maximalLateralSpeed,
                                                     const double & maximalAngularSpeed):
  longitudinalSpeed(makeLongitudinalSpeedCommandLimits(minimalLongitudinalSpeed,
                                                       maximalLongidudinalSpeed)),
  lateralSpeed(makeSymmetricCommandLimits(maximalLateralSpeed)),
  angularSpeed(makeSymmetricCommandLimits(maximalAngularSpeed))
{
}

//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const OmniSteeringCommandLimits & limits)
{
  os << "Command limits : " << std::endl;
  os << " longitudinal speed : [" <<limits.longitudinalSpeed.lower() << " "<< limits.longitudinalSpeed.upper()<<"]" << std::endl;
  os << " lateral speed : [" <<limits.lateralSpeed.lower() << " "<< limits.lateralSpeed.upper()<<"]"<<std::endl;
  os << " angular speed : [" <<limits.angularSpeed.lower() << " "<< limits.angularSpeed.upper()<<"]";
  return os;
}

}//end romea
