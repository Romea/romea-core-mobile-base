#ifndef romea_SkidSteeringCommand_hpp
#define romea_SkidSteeringCommand_hpp

#include "SkidSteeringCommandLimits.hpp"

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

}//end romea
#endif
