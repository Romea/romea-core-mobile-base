#ifndef romea_OmniSteeringCommand_hpp
#define romea_OmniSteeringCommand_hpp

//romea
#include "OmniSteeringCommandLimits.hpp"

namespace romea {

struct OmniSteeringCommand
{
  OmniSteeringCommand();

  OmniSteeringCommand(const double & longitudinalSpeed,
                      const double & lateralSpeed,
                      const double & angularSpeed);

  double longitudinalSpeed;
  double lateralSpeed;
  double angularSpeed;

};

std::ostream& operator<<(std::ostream& os, const OmniSteeringCommand & command);

OmniSteeringCommand clamp(const OmniSteeringCommand & command,
                          const OmniSteeringCommandLimits & limits);

bool isValid(const OmniSteeringCommand & commaand);

}//end romea
#endif
