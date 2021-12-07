//romea
#include "romea_core_odo/kinematic/axle_steering/OneAxleSteeringConstraints.hpp"

//std
#include <limits>
#include <cassert>
#include <cmath>

namespace romea {

//--------------------------------------------------------------------------
OneAxleSteeringConstraints::OneAxleSteeringConstraints():
  OneAxleSteeringConstraints(-std::numeric_limits<double>::max(),
                             std::numeric_limits<double>::max(),
                             M_PI_2)

{
}


//--------------------------------------------------------------------------
OneAxleSteeringConstraints::OneAxleSteeringConstraints(const double & minimalLinearSpeed,
                                                       const double & maximalLinearSpeed,
                                                       const double & maximalAbsoluteSteeringAngle):
  minimalLinearSpeed_(minimalLinearSpeed),
  maximalLinearSpeed_(maximalLinearSpeed),
  maximalAbsoluteSteeringAngle_(maximalAbsoluteSteeringAngle)
{
  assert(maximalLinearSpeed_>=0);
  assert(minimalLinearSpeed_<=0);
  assert(maximalAbsoluteSteeringAngle_>=0);
}

//--------------------------------------------------------------------------
void OneAxleSteeringConstraints::setMinimalLinearSpeed(const double & minimalLinearSpeed)
{
  assert(minimalLinearSpeed<=0);
  minimalLinearSpeed_=minimalLinearSpeed;
}

//--------------------------------------------------------------------------
void OneAxleSteeringConstraints::setMaximalLinearSpeed(const double & maximalLinearSpeed)
{
  assert(maximalLinearSpeed>=0);
  maximalLinearSpeed_=maximalLinearSpeed;

}

//--------------------------------------------------------------------------
void OneAxleSteeringConstraints::setMaximalAbsoluteSteeringAngle(const double & maximalAbsoluteSteeringAngle)
{
  assert(maximalAbsoluteSteeringAngle>=0);
  maximalAbsoluteSteeringAngle_=maximalAbsoluteSteeringAngle;
}

//--------------------------------------------------------------------------
const double & OneAxleSteeringConstraints::getMinimalLinearSpeed() const
{
  return minimalLinearSpeed_;
}

//--------------------------------------------------------------------------
const double & OneAxleSteeringConstraints::getMaximalLinearSpeed() const
{
  return maximalLinearSpeed_;
}

//--------------------------------------------------------------------------
const double & OneAxleSteeringConstraints::getMaximalAbsoluteSteeringAngle() const
{
  return maximalAbsoluteSteeringAngle_;
}

}//end romea

