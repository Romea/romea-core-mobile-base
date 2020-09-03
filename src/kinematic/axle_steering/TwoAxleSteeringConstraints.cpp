//romea
#include "romea_odo/kinematic/axle_steering/TwoAxleSteeringConstraints.hpp"

//std
#include <limits>
#include <cassert>
#include <cmath>

namespace romea {

//--------------------------------------------------------------------------
TwoAxleSteeringConstraints::TwoAxleSteeringConstraints():
  TwoAxleSteeringConstraints(-std::numeric_limits<double>::max(),
                             std::numeric_limits<double>::max(),
                             M_PI_2,
                             M_PI_2)

{

}


//--------------------------------------------------------------------------
TwoAxleSteeringConstraints::TwoAxleSteeringConstraints(const double & minimalLinearSpeed,
                                                       const double & maximalLinearSpeed,
                                                       const double & maximalAbsoluteFrontSteeringAngle,
                                                       const double & maximalAbsoluteRearSteeringAngle):
  minimalLinearSpeed_(minimalLinearSpeed),
  maximalLinearSpeed_(maximalLinearSpeed),
  maximalAbsoluteFrontSteeringAngle_(maximalAbsoluteFrontSteeringAngle),
  maximalAbsoluteRearSteeringAngle_(maximalAbsoluteRearSteeringAngle)

{

}

//--------------------------------------------------------------------------
void TwoAxleSteeringConstraints::setMinimalLinearSpeed(const double & minimalLinearSpeed)
{
  assert(minimalLinearSpeed<=0);
  minimalLinearSpeed_=minimalLinearSpeed;
}

//--------------------------------------------------------------------------
void TwoAxleSteeringConstraints::setMaximalLinearSpeed(const double & maximalLinearSpeed)
{
  assert(maximalLinearSpeed>=0);
  maximalLinearSpeed_=maximalLinearSpeed;

}

//--------------------------------------------------------------------------
void TwoAxleSteeringConstraints::setMaximalAbsoluteFrontSteeringAngle(const double & maximalAbsoluteFrontSteeringAngle)
{
  assert(maximalAbsoluteFrontSteeringAngle>=0);
  maximalAbsoluteFrontSteeringAngle_=maximalAbsoluteFrontSteeringAngle;
}

//--------------------------------------------------------------------------
void TwoAxleSteeringConstraints::setMaximalAbsoluteRearSteeringAngle(const double & maximalAbsoluteRearSteeringAngle)
{
  assert(maximalAbsoluteRearSteeringAngle>=0);
  maximalAbsoluteRearSteeringAngle_=maximalAbsoluteRearSteeringAngle;
}

//--------------------------------------------------------------------------
const double & TwoAxleSteeringConstraints::getMinimalLinearSpeed() const
{
  return minimalLinearSpeed_;
}

//--------------------------------------------------------------------------
const double & TwoAxleSteeringConstraints::getMaximalLinearSpeed() const
{
  return maximalLinearSpeed_;
}

//--------------------------------------------------------------------------
const double & TwoAxleSteeringConstraints::getMaximalAbsoluteFrontSteeringAngle() const
{
  return maximalAbsoluteFrontSteeringAngle_;
}

//--------------------------------------------------------------------------
const double & TwoAxleSteeringConstraints::getMaximalAbsoluteRearSteeringAngle() const
{
  return maximalAbsoluteRearSteeringAngle_;
}

}//end romea

