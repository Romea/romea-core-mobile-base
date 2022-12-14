#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_SKIDSTEERINGCOMMAND_HPP
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_SKIDSTEERINGCOMMAND_HPP

#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommandLimits.hpp"

namespace romea {

struct SkidSteeringCommand
{
  SkidSteeringCommand();

  SkidSteeringCommand(const double &longitudinalSpeed,
                      const double &angularSpeed);

  double longitudinalSpeed;
  double angularSpeed;
};

std::ostream& operator<<(std::ostream& os, const SkidSteeringCommand & command);

SkidSteeringCommand clamp(const SkidSteeringCommand & command,
                          const SkidSteeringCommandLimits & limits);


bool isValid(const SkidSteeringCommand & command);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_SKIDSTEERINGCOMMAND_HPP
