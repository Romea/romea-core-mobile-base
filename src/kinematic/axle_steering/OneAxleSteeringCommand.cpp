#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

namespace romea {

//-----------------------------------------------------------------------------
OneAxleSteeringCommand::OneAxleSteeringCommand():
  longitudinalSpeed(0.),
  steeringAngle(0.)
{
}

//-----------------------------------------------------------------------------
OneAxleSteeringCommand::OneAxleSteeringCommand(const double & longitudinalSpeed,
                                               const double & steeringAngle):
  longitudinalSpeed(longitudinalSpeed),
  steeringAngle(steeringAngle)
{

}

//-----------------------------------------------------------------------------
OneAxleSteeringCommand clamp(const OneAxleSteeringCommand & command,
                             const OneAxleSteeringCommandLimits & limits)
{
  OneAxleSteeringCommand clamped_command = command;
  clamped_command.longitudinalSpeed=clamp(command.longitudinalSpeed,limits.longitudinalSpeed);
  clamped_command.steeringAngle=clamp(command.steeringAngle,limits.steeringAngle);
  return clamped_command;
}

////-----------------------------------------------------------------------------
//KinematicCommand toKinematicCommand(const OneAxleSteeringCommand & command,const double & wheelBase)
//{
//  double instantaneousCurvature =OneAxleSteeringKinematic::computeInstantaneousCurvature(command.steeringAngle,wheelBase);

//  KinematicCommand convertedCommand;
//  convertedCommand.speed=command.longitudinalSpeed;
//  convertedCommand.beta=0;
//  convertedCommand.instantaneousCurvature=instantaneousCurvature;
//  convertedCommand.angularSpeed= instantaneousCurvature*command.longitudinalSpeed;

//  return convertedCommand;
//}

////-----------------------------------------------------------------------------
//KinematicCommand toKinematicCommand(const OneAxleSteeringCommand & command,
//                                    const Kinematic & kinematic)
//{
//  return toKinematicCommand(command,kinematic.getWheelBase("wheelbase"));
//}

////-----------------------------------------------------------------------------
//OneAxleSteeringCommand toOneAxleSteeringCommand(const KinematicCommand & command,
//                                                const double &wheelBase)
//{
//    assert(command.beta<std::numeric_limits<double>::epsilon());
//    OneAxleSteeringCommand convertedCommand;
//    convertedCommand.longitudinalSpeed =command.speed;
//    convertedCommand.steeringAngle=std::atan(command.instantaneousCurvature*wheelBase);
//    return convertedCommand;
//}

////-----------------------------------------------------------------------------
//OneAxleSteeringCommand toOneAxleSteeringCommand(const KinematicCommand & command,
//                                                const Kinematic & kinematic)
//{
//  return toOneAxleSteeringCommand(command,kinematic.getWheelBase("wheelbase"));
//}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const OneAxleSteeringCommand & command)
{
  os<<" OneAxleSteeringCommand command   "<<std::endl;;
  os<<" command longitudinal speed  " << command.longitudinalSpeed << std::endl;
  os<<" command front steering angle " << command.steeringAngle << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
bool isValid(const OneAxleSteeringCommand & command)
{
  return std::isfinite(command.longitudinalSpeed) &&
      std::isfinite(command.steeringAngle);
}


}//end romea

