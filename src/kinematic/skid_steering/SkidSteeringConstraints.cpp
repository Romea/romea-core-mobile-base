//romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringConstraints.hpp"
#include <romea_core_common/math/Algorithm.hpp>

//std
#include <limits>

namespace romea {


//--------------------------------------------------------------------------
SkidSteeringConstraints::SkidSteeringConstraints():
  SkidSteeringConstraints(-std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max())
{

}


//--------------------------------------------------------------------------
SkidSteeringConstraints::SkidSteeringConstraints(const double & minimalLinearSpeed,
                                                 const double &maximalLinearSpeed,
                                                 const double & maximalAbsoluteAngularSpeed):
  minimalLinearSpeed_(minimalLinearSpeed),
  maximalLinearSpeed_(maximalLinearSpeed),
  maximalAbsoluteAngularSpeed_(maximalAbsoluteAngularSpeed)
{

}

//--------------------------------------------------------------------------
void SkidSteeringConstraints::setMinimalLinearSpeed(const double & minimalLinearSpeed)
{
  assert(minimalLinearSpeed<=0);
  minimalLinearSpeed_=minimalLinearSpeed;
}

//--------------------------------------------------------------------------
void SkidSteeringConstraints::setMaximalLinearSpeed(const double & maximalLinearSpeed)
{
  assert(maximalLinearSpeed>=0);
  maximalLinearSpeed_=maximalLinearSpeed;

}

//--------------------------------------------------------------------------
void SkidSteeringConstraints::setMaximalAbsoluteAngularSpeed(const double & maximalAbsoluteAngularSpeed)
{
  assert(maximalAbsoluteAngularSpeed>=0);
  maximalAbsoluteAngularSpeed_=maximalAbsoluteAngularSpeed;
}

//--------------------------------------------------------------------------
const double & SkidSteeringConstraints::getMinimalLinearSpeed() const
{
  return minimalLinearSpeed_;
}

//--------------------------------------------------------------------------
const double & SkidSteeringConstraints::getMaximalLinearSpeed() const
{
  return maximalLinearSpeed_;
}

//--------------------------------------------------------------------------
const double & SkidSteeringConstraints::getMaximalAbsoluteAngularSpeed() const
{
  return maximalAbsoluteAngularSpeed_;
}

}//end romea

