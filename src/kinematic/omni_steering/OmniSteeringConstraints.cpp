//romea
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringConstraints.hpp"

//std
#include <limits>
#include <cassert>

namespace romea {


//--------------------------------------------------------------------------
OmniSteeringConstraints::OmniSteeringConstraints():
  OmniSteeringConstraints(-std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max())
{

}

//--------------------------------------------------------------------------
OmniSteeringConstraints::OmniSteeringConstraints(const double & minimalLongitudinalSpeed,
                                                 const double & maximalLongitudinalSpeed,
                                                 const double & maximalAbsoluteLateralSpeed,
                                                 const double & maximalAbsoluteAngularSpeed):
  minimalLongitudinalSpeed_(minimalLongitudinalSpeed),
  maximalLongitudinalSpeed_(maximalLongitudinalSpeed),
  maximalAbsoluteLateralSpeed_(maximalAbsoluteLateralSpeed),
  maximalAbsoluteAngularSpeed_(maximalAbsoluteAngularSpeed)
{

}


//--------------------------------------------------------------------------
void OmniSteeringConstraints::setMinimalLongitudinalSpeed(const double & minimalLinearSpeed)
{
  assert(minimalLinearSpeed<=0);
  minimalLongitudinalSpeed_=minimalLinearSpeed;
}

//--------------------------------------------------------------------------
void OmniSteeringConstraints::setMaximalLongitudinalSpeed(const double & maximalLinearSpeed)
{
  assert(maximalLinearSpeed>=0);
  maximalLongitudinalSpeed_=maximalLinearSpeed;

}

//--------------------------------------------------------------------------
void OmniSteeringConstraints::setMinimalAbsoluteLateralSpeed(const double & maximalAbsoluteLateralSpeed)
{
  assert(maximalAbsoluteLateralSpeed<=0);
  maximalAbsoluteLateralSpeed_=maximalAbsoluteLateralSpeed;
}


//--------------------------------------------------------------------------
void OmniSteeringConstraints::setMaximalAbsoluteAngularSpeed(const double & maximalAbsoluteAngularSpeed)
{
  assert(maximalAbsoluteAngularSpeed>=0);
  maximalAbsoluteAngularSpeed_=maximalAbsoluteAngularSpeed;
}


//--------------------------------------------------------------------------
const double & OmniSteeringConstraints::getMinimalLongitudinalSpeed() const
{
  return minimalLongitudinalSpeed_;
}

//--------------------------------------------------------------------------
const double & OmniSteeringConstraints::getMaximalLongitudinalSpeed() const
{
  return maximalLongitudinalSpeed_;
}

//--------------------------------------------------------------------------
const double & OmniSteeringConstraints::getMaximalAbsoluteLateralSpeed() const
{
  return maximalAbsoluteLateralSpeed_;
}

//--------------------------------------------------------------------------
const double & OmniSteeringConstraints::getMaximalAbsoluteAngularSpeed() const
{
  return maximalAbsoluteAngularSpeed_;
}

}//end romea

