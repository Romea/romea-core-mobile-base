// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__SKIDSTEERINGCOMMANDLIMITS_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__SKID_STEERING__SKIDSTEERINGCOMMANDLIMITS_HPP_

#include "romea_core_mobile_base/kinematic/CommandLimits.hpp"

namespace romea
{

struct SkidSteeringCommandLimits
{
  SkidSteeringCommandLimits();

  SkidSteeringCommandLimits(
    const double & minimalLongitudinalSpeed,
    const double & maximalLongidudinalSpeed,
    const double & maximalAngularSpeed);

  Interval1D<double> longitudinalSpeed;
  Interval1D<double> angularSpeed;
};

std::ostream & operator<<(std::ostream & os, const SkidSteeringCommandLimits & limits);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC__SKID_STEERING__SKIDSTEERINGCOMMANDLIMITS_HPP_
