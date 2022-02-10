//romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommandLimits.hpp"
#include <romea_core_common/math/Algorithm.hpp>

//std
#include <limits>

namespace romea {


//--------------------------------------------------------------------------
SkidSteeringCommandLimits::SkidSteeringCommandLimits():
  longitudinalSpeed(),
  angularSpeed()
{

}

//--------------------------------------------------------------------------
SkidSteeringCommandLimits::SkidSteeringCommandLimits(const double & minimalLongitudinalSpeed,
                                                     const double & maximalLongidudinalSpeed,
                                                     const double & maximalAngularSpeed):
  longitudinalSpeed(makeLongitudinalSpeedCommandLimits(minimalLongitudinalSpeed,
                                                       maximalLongidudinalSpeed)),
  angularSpeed(makeSymmetricCommandLimits(maximalAngularSpeed))
{

}

//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const SkidSteeringCommandLimits & limits)
{
  os << "Command limits : " << std::endl;
  os << " linear speed : [" <<limits.longitudinalSpeed.lower() << " "<< limits.longitudinalSpeed.upper()<<"]" << std::endl;
  os << " angular speed : [" <<limits.angularSpeed.lower() << " "<< limits.angularSpeed.upper()<<"]";
  return os;

}

}//end romea

