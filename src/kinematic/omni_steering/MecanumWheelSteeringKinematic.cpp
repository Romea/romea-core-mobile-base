//romea
#include "romea_core_mobile_base/kinematic/omni_steering/MecanumWheelSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

//std
#include <cmath>
#include <limits>
#include <algorithm>

namespace romea {


//--------------------------------------------------------------------------
MecanumWheelSteeringKinematic::Parameters::Parameters():
  wheelTrack(0),
  wheelbase(0),
  maximalWheelLinearSpeed(std::numeric_limits<double>::max()),
  maximalWheelLinearAcceleration(std::numeric_limits<double>::max()),
  wheelLinearSpeedVariance(0)
{

}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeFrontLeftWheelLinearSpeed(const double & longitidutinalSpeed,
                                                                       const double & lateralSpeed,
                                                                       const double & angularSpeed,
                                                                       const double & halfWheelbase,
                                                                       const double & halfTrack)
{
  return longitidutinalSpeed-lateralSpeed-angularSpeed*(halfWheelbase+halfTrack);
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeFrontRightWheelLinearSpeed(const double & longitidutinalSpeed,
                                                                        const double & lateralSpeed,
                                                                        const double & angularSpeed,
                                                                        const double & halfWheelbase,
                                                                        const double & halfTrack)
{
  return longitidutinalSpeed+lateralSpeed+angularSpeed*(halfWheelbase+halfTrack);
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeRearLeftWheelLinearSpeed(const double & longitidutinalSpeed,
                                                                      const double & lateralSpeed,
                                                                      const double & angularSpeed,
                                                                      const double & halfWheelbase,
                                                                      const double & halfTrack)
{
  return longitidutinalSpeed+lateralSpeed-angularSpeed*(halfWheelbase+halfTrack);
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeRearRightWheelLinearSpeed(const double & longitidutinalSpeed,
                                                                       const double & lateralSpeed,
                                                                       const double & angularSpeed,
                                                                       const double & halfWheelbase,
                                                                       const double & halfTrack)
{
  return longitidutinalSpeed-lateralSpeed+angularSpeed*(halfWheelbase+halfTrack);
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeLongitudinalSpeed(const double & frontLeftWheelSpeed,
                                                               const double & frontRightWheelSpeed,
                                                               const double & rearLeftWheelSpeed,
                                                               const double & rearRightWheelSpeed)
{
  return 0.25*(frontLeftWheelSpeed+frontRightWheelSpeed+
               rearLeftWheelSpeed+rearRightWheelSpeed);
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeLateralSpeed(const double & frontLeftWheelSpeed,
                                                          const double & frontRightWheelSpeed,
                                                          const double & rearLeftWheelSpeed,
                                                          const double & rearRightWheelSpeed)
{

  return  0.25* ((frontRightWheelSpeed+rearLeftWheelSpeed)-
                 (frontLeftWheelSpeed+rearRightWheelSpeed));
}

//--------------------------------------------------------------------------
double MecanumWheelSteeringKinematic::computeAngularSpeed(const double & frontLeftWheelSpeed,
                                                          const double & frontRightWheelSpeed,
                                                          const double & rearLeftWheelSpeed,
                                                          const double & rearRightWheelSpeed,
                                                          const double & halfWheelbase,
                                                          const double & halfTrack)
{
  return  0.25 *((frontRightWheelSpeed+rearRightWheelSpeed)-
                 (rearLeftWheelSpeed+frontLeftWheelSpeed))/(halfWheelbase+halfTrack);

}

//--------------------------------------------------------------------------
OmniSteeringCommand clamp(const MecanumWheelSteeringKinematic::Parameters & parameters,
                          const OmniSteeringCommandLimits & userLimits,
                          const OmniSteeringCommand & command)
{
  double alpha = parameters.wheelTrack+parameters.wheelbase;

  //Clamp angular speed
  double maximalAbsoluteAngularSpeed = 2*parameters.maximalWheelLinearSpeed/alpha;

  maximalAbsoluteAngularSpeed = std::min(maximalAbsoluteAngularSpeed,
                                         userLimits.angularSpeed.upper());

  double angularSpeed = romea::clamp(command.angularSpeed,
                                     -maximalAbsoluteAngularSpeed,
                                     maximalAbsoluteAngularSpeed);

  //Clamp lateral speed
  double maximalAbsoluteSpeed= parameters.maximalWheelLinearSpeed - std::abs(angularSpeed)*alpha/2.0;

  double maximalAbsboluteLateralSpeed = std::min(maximalAbsoluteSpeed,
                                                 userLimits.lateralSpeed.upper());

  double lateralSpeed = romea::clamp(command.lateralSpeed,
                                     -maximalAbsboluteLateralSpeed,
                                     maximalAbsboluteLateralSpeed);

  //Clamp longitudinal speed
  double longitudinalSpeed = command.longitudinalSpeed;

  double maximalAbsoluteLongitudinalSpeed = maximalAbsoluteSpeed-std::abs(lateralSpeed);

  double minimalLongitudinalSpeed = std::max(-maximalAbsoluteLongitudinalSpeed,userLimits.longitudinalSpeed.lower());

  longitudinalSpeed = std::max(longitudinalSpeed,minimalLongitudinalSpeed);

  double maximalLongitudinalSpeed = std::min( maximalAbsoluteLongitudinalSpeed,userLimits.longitudinalSpeed.upper());

  longitudinalSpeed = std::min(longitudinalSpeed,maximalLongitudinalSpeed);


  OmniSteeringCommand clampedCommand;
  clampedCommand.longitudinalSpeed=longitudinalSpeed;
  clampedCommand.lateralSpeed=lateralSpeed;
  clampedCommand.angularSpeed=angularSpeed;
  return clampedCommand;

}


//--------------------------------------------------------------------------
OmniSteeringCommand clamp(const MecanumWheelSteeringKinematic::Parameters & parameters,
                          const OmniSteeringCommand & previousCommand,
                          const OmniSteeringCommand & currentCommand,
                          const double & dt)
{
  double alpha = parameters.wheelTrack+parameters.wheelbase;

  //Clamp angular speed
  double maximalAbsoluteAngularSpeed = 2*parameters.maximalWheelLinearAcceleration/alpha;

  double angularAcceleration = currentCommand.angularSpeed-previousCommand.angularSpeed;

  angularAcceleration = romea::clamp(angularAcceleration,
                                     -maximalAbsoluteAngularSpeed,
                                     maximalAbsoluteAngularSpeed);

  //Clamp lateral speed
  double maximalAbsoluteLateralAcceleration = parameters.maximalWheelLinearAcceleration - std::abs(angularAcceleration)*alpha/2.0;

  double lateralAcceleration = currentCommand.lateralSpeed-previousCommand.lateralSpeed;

  lateralAcceleration = romea::clamp(lateralAcceleration,
                                     -maximalAbsoluteLateralAcceleration,
                                     maximalAbsoluteLateralAcceleration);

  //Clamp longitudinal speed

  double maximalAbsoluteLongitudinalAcceleration = maximalAbsoluteLateralAcceleration-std::abs(lateralAcceleration);

  double longitudinalAcceleration = currentCommand.longitudinalSpeed - previousCommand.longitudinalSpeed;

  longitudinalAcceleration = romea::clamp(longitudinalAcceleration,
                                          -maximalAbsoluteLongitudinalAcceleration,
                                          maximalAbsoluteLongitudinalAcceleration);


  //return command
  OmniSteeringCommand clampedCommand;
  clampedCommand.longitudinalSpeed=previousCommand.longitudinalSpeed+longitudinalAcceleration*dt;
  clampedCommand.lateralSpeed=previousCommand.lateralSpeed+lateralAcceleration*dt;
  clampedCommand.angularSpeed=previousCommand.angularSpeed+angularAcceleration*dt;
  return clampedCommand;

}

}//end romea
