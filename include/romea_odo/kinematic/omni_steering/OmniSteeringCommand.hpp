#ifndef romea_OmniSteeringCommand_hpp
#define romea_OmniSteeringCommand_hpp

//romea
#include "OmniSteeringConstraints.hpp"

namespace romea {

struct OmniSteeringCommand
{
  OmniSteeringCommand();

  double longitudinalSpeed;
  double lateralSpeed;
  double angularSpeed;

};

std::ostream& operator<<(std::ostream& os, const OmniSteeringCommand & command);

OmniSteeringCommand clamp(const OmniSteeringCommand & command,
                          const OmniSteeringConstraints & constraints);

}//end romea
#endif
