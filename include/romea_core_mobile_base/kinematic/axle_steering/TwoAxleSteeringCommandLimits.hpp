// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__TWOAXLESTEERINGCOMMANDLIMITS_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__TWOAXLESTEERINGCOMMANDLIMITS_HPP_

#include "romea_core_mobile_base/kinematic/CommandLimits.hpp"

namespace romea
{


struct TwoAxleSteeringCommandLimits
{
  TwoAxleSteeringCommandLimits();

  TwoAxleSteeringCommandLimits(
    const double & minimalLongitudinalSpeed,
    const double & maximalLongidudinalSpeed,
    const double & maximalFrontSteeringAngle,
    const double & maximalRearSteeringAngle);

  Interval1D<double> longitudinalSpeed;
  Interval1D<double> frontSteeringAngle;
  Interval1D<double> rearSteeringAngle;

};

std::ostream & operator<<(std::ostream & os, const TwoAxleSteeringCommandLimits & limits);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__TWOAXLESTEERINGCOMMANDLIMITS_HPP_
