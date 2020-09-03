#ifndef romea_TwoAxleSteeringCommand_hpp
#define romea_TwoAxleSteeringCommand_hpp

//romea
#include "TwoAxleSteeringConstraints.hpp"

namespace romea {

struct TwoAxleSteeringCommand
{
  TwoAxleSteeringCommand();

  double longitudinalSpeed;
  double frontSteeringAngle;
  double rearSteeringAngle;

};

TwoAxleSteeringCommand clamp(const TwoAxleSteeringCommand & command,
                             const TwoAxleSteeringConstraints & constraints);

std::ostream& operator<<(std::ostream& os, const TwoAxleSteeringCommand & command);

}//end romea
#endif
