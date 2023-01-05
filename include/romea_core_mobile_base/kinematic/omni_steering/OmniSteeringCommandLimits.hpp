// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGCOMMANDLIMITS_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGCOMMANDLIMITS_HPP_

#include "romea_core_mobile_base/kinematic/CommandLimits.hpp"

namespace romea
{

struct OmniSteeringCommandLimits
{
  OmniSteeringCommandLimits();

  OmniSteeringCommandLimits(
    const double & minimalLongitudinalSpeed,
    const double & maximalLongidudinalSpeed,
    const double & maximalLateralSpeed,
    const double & maximalAngularSpeed);

  Interval1D<double> longitudinalSpeed;
  Interval1D<double> lateralSpeed;
  Interval1D<double> angularSpeed;
};

std::ostream & operator<<(std::ostream & os, const OmniSteeringCommandLimits & limits);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGCOMMANDLIMITS_HPP_
