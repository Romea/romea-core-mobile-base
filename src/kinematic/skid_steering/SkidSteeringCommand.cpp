//romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp"
#include <romea_core_common/math/Algorithm.hpp>

namespace romea {


////-----------------------------------------------------------------------------
//KinematicCommand toKinematicCommand(const SkidSteeringCommand & command)
//{

//  auto instantaneousCurvature = safe_divide(command.angularSpeed,
//                                            command.longitudinalSpeed);

//  KinematicCommand convertedCommand;
//  convertedCommand.beta=0;
//  convertedCommand.speed=command.longitudinalSpeed;
//  convertedCommand.angularSpeed = command.angularSpeed;

//  if(instantaneousCurvature)
//  {
//    convertedCommand.instantaneousCurvature = *instantaneousCurvature;
//  }
//  else
//  {
//    convertedCommand.instantaneousCurvature = 0;
//  }

//  return convertedCommand;
//}

////-----------------------------------------------------------------------------
//KinematicCommand toKinematicCommand(const SkidSteeringCommand & command,const Kinematic & /*kinematic*/)
//{
//  return toKinematicCommand(command);
//}

////-----------------------------------------------------------------------------
//SkidSteeringCommand toSkidSteeringCommand(const KinematicCommand & command)
//{
//  assert(command.beta<std::numeric_limits<double>::epsilon());
//  SkidSteeringCommand convertedCommand;
//  convertedCommand.longitudinalSpeed=command.speed;
//  convertedCommand.angularSpeed=command.angularSpeed;
//  return convertedCommand;
//}

////----------------------------------------------------------------------------
//SkidSteeringCommand toSkidSteeringCommand(const KinematicCommand & command,const Kinematic & /*kinematic*/)
//{
//  return toSkidSteeringCommand(command);
//}


//-----------------------------------------------------------------------------
SkidSteeringCommand::SkidSteeringCommand():
  SkidSteeringCommand(0,0)
{

}

//-----------------------------------------------------------------------------
SkidSteeringCommand::SkidSteeringCommand(const double &longitudinalSpeed,
                                         const double &angularSpeed):
  longitudinalSpeed(longitudinalSpeed),
  angularSpeed(angularSpeed)
{
}

//-----------------------------------------------------------------------------
SkidSteeringCommand clamp(const SkidSteeringCommand & command,
                          const SkidSteeringCommand & lowerBound,
                          const SkidSteeringCommand & upperBound)
{
  SkidSteeringCommand clamped_command = command;

  clamped_command.longitudinalSpeed=clamp(clamped_command.longitudinalSpeed,
                                          lowerBound.longitudinalSpeed,
                                          upperBound.longitudinalSpeed);

  clamped_command.angularSpeed=clamp(clamped_command.angularSpeed,
                                     lowerBound.angularSpeed,
                                     upperBound.angularSpeed);

  return clamped_command;
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const SkidSteeringCommand & command)
{
  os<<" SkidSteering command   "<<std::endl;;
  os<<" command longitudinal speed  " << command.longitudinalSpeed << std::endl;
  os<<" command angular speed " << command.angularSpeed << std::endl;
  return os;
}

//-----------------------------------------------------------------------------
SkidSteeringCommand clamp(const SkidSteeringCommand & command,
                          const SkidSteeringConstraints & constraints)
{
  SkidSteeringCommand clamped_command = command;
  clamped_command.longitudinalSpeed=std::max(clamped_command.longitudinalSpeed,-constraints.getMinimalLinearSpeed());
  clamped_command.longitudinalSpeed=std::min(clamped_command.longitudinalSpeed,constraints.getMaximalLinearSpeed());
  clamped_command.angularSpeed=std::max(clamped_command.angularSpeed,-constraints.getMaximalAbsoluteAngularSpeed());
  clamped_command.angularSpeed=std::min(clamped_command.angularSpeed, constraints.getMaximalAbsoluteAngularSpeed());
  return clamped_command;
}

//-----------------------------------------------------------------------------
bool isValid(const SkidSteeringCommand & command)
{
    return std::isfinite(command.longitudinalSpeed) &&
            std::isfinite(command.angularSpeed);
}


}//end romea
