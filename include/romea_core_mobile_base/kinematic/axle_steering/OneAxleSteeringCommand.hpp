// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGCOMMAND_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGCOMMAND_HPP_

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommandLimits.hpp"

namespace romea
{

struct OneAxleSteeringCommand
{
  OneAxleSteeringCommand();

  OneAxleSteeringCommand(
    const double & longitudinalSpeed,
    const double & steeringAngle);

  double longitudinalSpeed;
  double steeringAngle;
};

OneAxleSteeringCommand clamp(
  const OneAxleSteeringCommand & command,
  const OneAxleSteeringCommandLimits & limits);

std::ostream & operator<<(std::ostream & os, const OneAxleSteeringCommand & command);

bool isValid(const OneAxleSteeringCommand & command);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__ONEAXLESTEERINGCOMMAND_HPP_
