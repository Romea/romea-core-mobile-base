#ifndef romea_OneAxleSteeringCommand_hpp
#define romea_OneAxleSteeringCommand_hpp

//romea
#include "OneAxleSteeringConstraints.hpp"

namespace romea {

struct OneAxleSteeringCommand
{

  OneAxleSteeringCommand();

  double longitudinalSpeed;
  double steeringAngle;

};

OneAxleSteeringCommand clamp(const OneAxleSteeringCommand & command,
                             const OneAxleSteeringConstraints & constraints);

std::ostream& operator<<(std::ostream& os, const OneAxleSteeringCommand & command);

}//end romea
#endif
