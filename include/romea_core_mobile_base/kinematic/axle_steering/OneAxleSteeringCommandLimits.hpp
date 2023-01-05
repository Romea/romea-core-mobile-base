// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGCOMMANDLIMITS_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGCOMMANDLIMITS_HPP_

#include "romea_core_mobile_base/kinematic/CommandLimits.hpp"

namespace romea
{


struct OneAxleSteeringCommandLimits
{
  OneAxleSteeringCommandLimits();

  OneAxleSteeringCommandLimits(
    const double & minimalLongitudinalSpeed,
    const double & maximalLongidudinalSpeed,
    const double & maximalSteeringAngle);

  Interval1D<double> longitudinalSpeed;
  Interval1D<double> steeringAngle;
};

std::ostream & operator<<(std::ostream & os, const OneAxleSteeringCommandLimits & limits);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGCOMMANDLIMITS_HPP_
