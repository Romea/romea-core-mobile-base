#include "romea_odo/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"
#include "romea_odo/kinematic/axle_steering/TwoAxleSteeringCommand.hpp"
#include "romea_odo/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include <romea_common/math/Algorithm.hpp>

namespace romea {



////-----------------------------------------------------------------------------
//KinematicCommand toKinematicCommand(const TwoAxleSteeringCommand & command,
//                                    const double & frontWheelBase,
//                                    const double & rearWheeBase)
//{
//  double othogonalInstantaneousCurvature=TwoAxleSteeringKinematic::
//      computeOrthogonalInstantaneousCurvature(std::tan(command.frontSteeringAngle),
//                                              std::tan(command.rearSteeringAngle),
//                                              frontWheelBase,
//                                              rearWheeBase);

//  double beta= TwoAxleSteeringKinematic::computeBeta(std::tan(command.frontSteeringAngle),
//                                                     std::tan(command.rearSteeringAngle),
//                                                     frontWheelBase,
//                                                     rearWheeBase);

//  double instantaneousCurvature = othogonalInstantaneousCurvature*std::cos(beta);


//  KinematicCommand convertedCommand;
//  convertedCommand.speed=command.longitudinalSpeed/std::cos(beta);
//  convertedCommand.beta=beta;
//  convertedCommand.angularSpeed =othogonalInstantaneousCurvature*command.longitudinalSpeed;
//  convertedCommand.instantaneousCurvature =instantaneousCurvature;
//  return convertedCommand;
//}


////-----------------------------------------------------------------------------
//KinematicCommand toKinematicCommand(const TwoAxleSteeringCommand & command,
//                                    const Kinematic & kinematic)
//{
//  return toKinematicCommand(command,
//                            kinematic.getWheelBase("front_wheelbase"),
//                            kinematic.getWheelBase("rear_wheelbase"));
//}


////-----------------------------------------------------------------------------
//TwoAxleSteeringCommand toTwoAxleSteeringCommand(const KinematicCommand & command,
//                                                const double & frontWheelBase,
//                                                const double & rearWheelBase)
//{
//  double frontSteeringAngle = FourWheelSteeringKinematic::
//      computeFrontSteeringAngle(command.instantaneousCurvature,
//                                frontWheelBase,
//                                command.beta);

//  double rearSteeringAngle = FourWheelSteeringKinematic::
//      computeRearSteeringAngle(command.instantaneousCurvature,
//                               rearWheelBase,
//                               command.beta);

//  TwoAxleSteeringCommand convertedCommand;
//  convertedCommand.longitudinalSpeed=command.speed*std::cos(command.beta);
//  convertedCommand.frontSteeringAngle=frontSteeringAngle;
//  convertedCommand.rearSteeringAngle=rearSteeringAngle;
//  return convertedCommand;
//}

////-----------------------------------------------------------------------------
//TwoAxleSteeringCommand toTwoAxleSteeringCommand(const KinematicCommand & command,
//                                                const Kinematic & kinematic)
//{
//  return toTwoAxleSteeringCommand(command,
//                                  kinematic.getWheelBase("front_wheelbase"),
//                                  kinematic.getWheelBase("rear_wheelbase"));
//}

//-----------------------------------------------------------------------------
TwoAxleSteeringCommand::TwoAxleSteeringCommand():
  longitudinalSpeed(0.),
  frontSteeringAngle(0.),
  rearSteeringAngle(0.)
{
}

//-----------------------------------------------------------------------------
TwoAxleSteeringCommand clamp(const TwoAxleSteeringCommand & command,
                             const TwoAxleSteeringConstraints & constraints)
{
  TwoAxleSteeringCommand clamped_command = command;
  clamped_command.longitudinalSpeed=std::max(clamped_command.longitudinalSpeed,constraints.getMinimalLinearSpeed());
  clamped_command.longitudinalSpeed=std::min(clamped_command.longitudinalSpeed,constraints.getMaximalLinearSpeed());
  clamped_command.frontSteeringAngle=std::max(clamped_command.frontSteeringAngle,-constraints.getMaximalAbsoluteFrontSteeringAngle());
  clamped_command.frontSteeringAngle=std::min(clamped_command.frontSteeringAngle, constraints.getMaximalAbsoluteFrontSteeringAngle());
  clamped_command.rearSteeringAngle=std::max(clamped_command.rearSteeringAngle,-constraints.getMaximalAbsoluteRearSteeringAngle());
  clamped_command.rearSteeringAngle=std::min(clamped_command.rearSteeringAngle, constraints.getMaximalAbsoluteRearSteeringAngle());
  return clamped_command;
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const TwoAxleSteeringCommand & command)
{
  os<<" TwoAxleSteering Command   "<<std::endl;;
  os<<" command longitudinal speed  " << command.longitudinalSpeed << std::endl;
  os<<" command front steering angle " << command.frontSteeringAngle << std::endl;
  os<<" command rear steering angle " << command.rearSteeringAngle << std::endl;
  return os;
}



}//end romea

