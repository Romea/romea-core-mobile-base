#include "romea_odo/kinematic/omni_steering/OmniSteeringCommand.hpp"
#include <romea_common/math/Algorithm.hpp>

namespace romea {

//-----------------------------------------------------------------------------
OmniSteeringCommand::OmniSteeringCommand():
  longitudinalSpeed(0.),
  lateralSpeed(0.),
  angularSpeed(0.)
{

}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const OmniSteeringCommand & command)
{
  os<<" OmniSteering command   "<<std::endl;;
  os<<" command longitudinal speed  " << command.longitudinalSpeed << std::endl;
  os<<" command lateral speed  " << command.lateralSpeed << std::endl;
  os<<" command angular speed " << command.angularSpeed << std::endl;
  return os;
}


//-----------------------------------------------------------------------------
OmniSteeringCommand clamp(const OmniSteeringCommand & command,
                          const OmniSteeringConstraints & constraints)
{
  OmniSteeringCommand clampedCommand;
  clampedCommand.longitudinalSpeed=clamp(command.longitudinalSpeed,constraints.getMinimalLongitudinalSpeed(),constraints.getMaximalLongitudinalSpeed());
  clampedCommand.lateralSpeed=clamp(command.lateralSpeed,-constraints.getMaximalAbsoluteLateralSpeed(),constraints.getMaximalAbsoluteLateralSpeed());
  clampedCommand.lateralSpeed=clamp(command.angularSpeed,-constraints.getMaximalAbsoluteAngularSpeed(),constraints.getMaximalAbsoluteAngularSpeed());
  return clampedCommand;

}


}//end romea
