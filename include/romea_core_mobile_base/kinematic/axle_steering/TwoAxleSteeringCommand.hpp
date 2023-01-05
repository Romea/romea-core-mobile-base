// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__TWOAXLESTEERINGCOMMAND_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__TWOAXLESTEERINGCOMMAND_HPP_

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommandLimits.hpp"

namespace romea
{

struct TwoAxleSteeringCommand
{
  TwoAxleSteeringCommand();

  TwoAxleSteeringCommand(
    const double & longitudinalSpeed,
    const double & frontSteeringAngle,
    const double & rearSteeringAngle);


  double longitudinalSpeed;
  double frontSteeringAngle;
  double rearSteeringAngle;
};

TwoAxleSteeringCommand clamp(
  const TwoAxleSteeringCommand & command,
  const TwoAxleSteeringCommandLimits & limits);

std::ostream & operator<<(std::ostream & os, const TwoAxleSteeringCommand & command);

bool isValid(const TwoAxleSteeringCommand & command);

}  //  namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__AXLE_STEERING__TWOAXLESTEERINGCOMMAND_HPP_
