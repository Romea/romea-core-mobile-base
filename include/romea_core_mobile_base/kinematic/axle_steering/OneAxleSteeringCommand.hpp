#ifndef romea_OneAxleSteeringCommand_hpp
#define romea_OneAxleSteeringCommand_hpp

//romea
#include "OneAxleSteeringCommandLimits.hpp"

namespace romea {

struct OneAxleSteeringCommand
{

  OneAxleSteeringCommand();

  OneAxleSteeringCommand(const double & longitudinalSpeed,
                         const double & steeringAngle);

  double longitudinalSpeed;
  double steeringAngle;

};

OneAxleSteeringCommand clamp(const OneAxleSteeringCommand & command,
                             const OneAxleSteeringCommandLimits & limits);

std::ostream& operator<<(std::ostream& os, const OneAxleSteeringCommand & command);

bool isValid(const OneAxleSteeringCommand & command);

}//end romea
#endif
