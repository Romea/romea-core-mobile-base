// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGCOMMAND_HPP
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGCOMMAND_HPP

// romea
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommandLimits.hpp"

namespace romea
{

struct OmniSteeringCommand
{
  OmniSteeringCommand();

  OmniSteeringCommand(
    const double & longitudinalSpeed,
    const double & lateralSpeed,
    const double & angularSpeed);

  double longitudinalSpeed;
  double lateralSpeed;
  double angularSpeed;
};

std::ostream & operator<<(std::ostream & os, const OmniSteeringCommand & command);

OmniSteeringCommand clamp(
  const OmniSteeringCommand & command,
  const OmniSteeringCommandLimits & limits);

bool isValid(const OmniSteeringCommand & commaand);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__OMNISTEERINGCOMMAND_HPP
