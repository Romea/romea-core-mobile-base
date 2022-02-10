#ifndef romea_TwoAxleSteeringCommand_hpp
#define romea_TwoAxleSteeringCommand_hpp

//romea
#include "TwoAxleSteeringCommandLimits.hpp"

namespace romea {

struct TwoAxleSteeringCommand
{
  TwoAxleSteeringCommand();

  TwoAxleSteeringCommand(const double & longitudinalSpeed,
                         const double & frontSteeringAngle,
                         const double & rearSteeringAngle);


  double longitudinalSpeed;
  double frontSteeringAngle;
  double rearSteeringAngle;

};

TwoAxleSteeringCommand clamp(const TwoAxleSteeringCommand & command,
                             const TwoAxleSteeringCommandLimits & limits);

std::ostream& operator<<(std::ostream& os, const TwoAxleSteeringCommand & command);

bool isValid(const TwoAxleSteeringCommand & command);

}//end romea
#endif
