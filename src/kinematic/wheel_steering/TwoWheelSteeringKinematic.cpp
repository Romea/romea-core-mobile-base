//romea
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

//std
#include <cmath>
#include <limits>
#include <iostream>

namespace romea {


//--------------------------------------------------------------------------
TwoWheelSteeringKinematic::Parameters::Parameters():
  frontWheelBase(0),
  rearWheelBase(0),
  frontWheelTrack(0),
  rearWheelTrack(0),
  frontHubCarrierOffset(0),
  rearHubCarrierOffset(0),
  frontMaximalWheelSpeed(std::numeric_limits<double>::max()),
  rearMaximalWheelSpeed(std::numeric_limits<double>::max()),
  maximalWheelAcceleration(std::numeric_limits<double>::max()),
  maximalWheelAngle(M_PI_2),
  maximalWheelAngularSpeed(std::numeric_limits<double>::max()),
  wheelSpeedVariance(0),
  wheelAngleVariance(0)

{

}


//--------------------------------------------------------------------------
double TwoWheelSteeringKinematic::computeInstantaneousCurvature(const double & leftInstantaneousCurvature,
                                                                const double & rightInstantaneousCurvature,
                                                                const double & track)
{
  return 0.5*(leftInstantaneousCurvature/(1+leftInstantaneousCurvature*track/2)+
              rightInstantaneousCurvature/(1-rightInstantaneousCurvature*track/2));
}

//--------------------------------------------------------------------------
double TwoWheelSteeringKinematic::computeInstantaneousCurvature(const double & leftWheelAngle,
                                                                const double & rightWheelAngle,
                                                                const double & wheelbase,
                                                                const double & track)
{
  double leftInstantaneousCurvature=OneAxleSteeringKinematic::
      computeInstantaneousCurvature(leftWheelAngle,wheelbase);
  double rightInstantaneousCurvature=OneAxleSteeringKinematic::
      computeInstantaneousCurvature(rightWheelAngle,wheelbase);

  return computeInstantaneousCurvature(leftInstantaneousCurvature,
                                       rightInstantaneousCurvature,
                                       track);
}

//--------------------------------------------------------------------------
double TwoWheelSteeringKinematic::computeSteeringAngle(const double & leftWheelAngle,
                                                       const double & rightWheelAngle,
                                                       const double & wheelbase,
                                                       const double & track)
{
  double instantaneousCurvature = computeInstantaneousCurvature(leftWheelAngle,
                                                                rightWheelAngle,
                                                                wheelbase,
                                                                track);
  return OneAxleSteeringKinematic::computeSteeringAngle(instantaneousCurvature,
                                                        wheelbase);

}



//--------------------------------------------------------------------------
double TwoWheelSteeringKinematic::computeLeftWheelAngle(const double & tanSteeringAngle,
                                                        const double & instantaneousCurvature,
                                                        const double & halfTrack)
{
  return std::atan(tanSteeringAngle/(1-instantaneousCurvature*halfTrack));
}

//--------------------------------------------------------------------------
double TwoWheelSteeringKinematic::computeRightWheelAngle(const double & tanSteeringAngle,
                                                         const double & instantaneousCurvature,
                                                         const double & halfTrack)
{

  return std::atan(tanSteeringAngle/(1+instantaneousCurvature*halfTrack));
}

//--------------------------------------------------------------------------
double TwoWheelSteeringKinematic::computeMaximalInstantaneousCurvature(const double wheelbase,
                                                                       const double halfTrack,
                                                                       const double & maximalWheelAngle)
{

  double maximalAbsoluteIntantaneousCurvature=OneAxleSteeringKinematic::
      computeInstantaneousCurvature(std::tan(maximalWheelAngle),wheelbase);

  return maximalAbsoluteIntantaneousCurvature= maximalAbsoluteIntantaneousCurvature
      /(1+maximalAbsoluteIntantaneousCurvature*halfTrack);
}

////--------------------------------------------------------------------------
//double maximalPermissibleLinearSpeed(const TwoWheelSteeringKinematic::Parameters & parameters,
//                                     const double & instantaneousCurvature)
//{
//  double wheelbase = parameters.rearWheelBase+parameters.frontWheelBase;

//  double maximalInstantaneousCurvature = TwoWheelSteeringKinematic::
//      computeMaximalInstantaneousCurvature(wheelbase,
//                                           parameters.frontTrack/2.,
//                                           parameters.maximalWheelAngle);

//  double maximalVirtualSteeringAngle = std::atan(maximalInstantaneousCurvature*wheelbase);


//  double frontMaximalLinearSpeed= OneAxleSteeringKinematic::
//      maximalPermissibleLinearSpeed(wheelbase,
//                                    parameters.frontTrack,
//                                    parameters.frontMaximalWheelSpeed,
//                                    maximalVirtualSteeringAngle,
//                                    parameters.frontHubCarrierOffset,
//                                    instantaneousCurvature);

//  double rearMaximalLinearSpeed= SkidSteeringKinematic::
//      maximalPermissibleLinearSpeed(parameters.rearTrack+2*parameters.rearHubCarrierOffset,
//                                    parameters.rearMaximalWheelSpeed,
//                                    instantaneousCurvature);

//  return std::min(frontMaximalLinearSpeed,rearMaximalLinearSpeed);
//}

////--------------------------------------------------------------------------
//double maximalPermissibleInstantaneousCurvature(const TwoWheelSteeringKinematic::Parameters & parameters,
//                                                const double & linearSpeed)
//{

//  double wheelbase = parameters.rearWheelBase+parameters.frontWheelBase;

//  double maximalInstantaneousCurvature = TwoWheelSteeringKinematic::
//      computeMaximalInstantaneousCurvature(wheelbase,
//                                           parameters.frontTrack/2.,
//                                           parameters.maximalWheelAngle);

//  double maximalVirtualSteeringAngle = std::atan(maximalInstantaneousCurvature*wheelbase);

//  double frontMaximalInstantaneousCurvature= OneAxleSteeringKinematic::
//      maximalPermissibleInstantaneousCurvature(wheelbase,
//                                               parameters.frontTrack,
//                                               parameters.frontMaximalWheelSpeed,
//                                               maximalVirtualSteeringAngle,
//                                               parameters.frontHubCarrierOffset,
//                                               linearSpeed);

//  double rearMaximalInstantaneousCurvature= SkidSteeringKinematic::
//      maximalPermissibleInstantaneousCurvature(parameters.rearTrack+2*parameters.rearHubCarrierOffset,
//                                               parameters.rearMaximalWheelSpeed,
//                                               linearSpeed);

//  return std::min(frontMaximalInstantaneousCurvature,rearMaximalInstantaneousCurvature);

//}

//--------------------------------------------------------------------------
OneAxleSteeringCommand clamp(const TwoWheelSteeringKinematic::Parameters & parameters,
                             const OneAxleSteeringCommandLimits & userLimits,
                             const OneAxleSteeringCommand & command)
{


  double wheelbase = parameters.rearWheelBase+parameters.frontWheelBase;

  double maximalInstantaneousCurvature =
      TwoWheelSteeringKinematic::computeMaximalInstantaneousCurvature(wheelbase,
                                                                      parameters.frontWheelTrack/2.,
                                                                      parameters.maximalWheelAngle);


  return OneAxleSteeringKinematic::clamp(wheelbase,
                                         parameters.frontWheelTrack/2.,
                                         parameters.rearWheelTrack/2.,
                                         parameters.frontHubCarrierOffset,
                                         parameters.rearHubCarrierOffset,
                                         std::atan(maximalInstantaneousCurvature*wheelbase),
                                         parameters.frontMaximalWheelSpeed,
                                         parameters.rearMaximalWheelSpeed,
                                         userLimits,
                                         command);
}

//--------------------------------------------------------------------------
OneAxleSteeringCommand clamp(const TwoWheelSteeringKinematic::Parameters & parameters,
                             const OneAxleSteeringCommand & previousCommand,
                             const OneAxleSteeringCommand & currentCommand,
                             const double & dt)
{
  double wheelbase = parameters.rearWheelBase+parameters.frontWheelBase;

  double tanSteeringAngle = std::tan(previousCommand.steeringAngle);
  double alpha = tanSteeringAngle *tanSteeringAngle;
  double beta = 1+tanSteeringAngle*parameters.frontWheelTrack/(2*wheelbase);

  double maximalSteeringAngularSpeed =parameters.maximalWheelAngularSpeed*
      (alpha + std::pow(beta,2.))/(1+alpha);

  return OneAxleSteeringKinematic::clamp(wheelbase,
                                         parameters.frontWheelTrack/2.,
                                         parameters.rearWheelTrack/2.,
                                         parameters.frontHubCarrierOffset,
                                         parameters.rearHubCarrierOffset,
                                         maximalSteeringAngularSpeed,
                                         parameters.maximalWheelAcceleration,
                                         previousCommand,
                                         currentCommand,
                                         dt);

}

////--------------------------------------------------------------------------
//KinematicCommand clamp(const KinematicCommand & command,
//                       const TwoWheelSteeringKinematic &kinematic,
//                       const KinematicConstraints & userLimits)
//{
//  //clamp curvature
//  double maximalAbsoluteIntantaneousCurvature= std::min(kinematic.computeMaximalInstantaneousCurvature(),
//                                                        userLimits.getMaximalAbsoluteInstantaneousCurvature());

//  double instantaneousCurvature =romea::clamp(command.instantaneousCurvature,
//                                              -maximalAbsoluteIntantaneousCurvature,
//                                              maximalAbsoluteIntantaneousCurvature);

//  //clamp linear speed
//  const double wheelBase= kinematic.getWheelBase("wheelbase");
//  const double frontTrack  = kinematic.getTrack("front_track");

//  double maximalAbsoluteLinearSpeed = std::numeric_limits<double>::max();

//  maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,
//                                        userLimits.getMaximalAbsoluteAngularSpeed()/
//                                        userLimits.getMaximalAbsoluteInstantaneousCurvature());

//  double maximalAbsoluteWheelAngle = std::atan(maximalAbsoluteIntantaneousCurvature*wheelBase);

//  maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,
//                                        kinematic.getMaximalWheelSpeed()*std::cos(maximalAbsoluteWheelAngle)/
//                                        (1 + maximalAbsoluteWheelAngle*frontTrack/2));

//  double minimalSpeed = std::max(userLimits.getMinimalSpeed(),
//                                 -maximalAbsoluteLinearSpeed);

//  double maximalSpeed = std::min(userLimits.getMaximalSpeed(),
//                                 maximalAbsoluteLinearSpeed);

//  double linearSpeed = romea::clamp(command.speed,
//                                    minimalSpeed,
//                                    maximalSpeed);

//  KinematicCommand clampedCommand;
//  clampedCommand.speed=linearSpeed;
//  clampedCommand.beta=0;
//  clampedCommand.angularSpeed =instantaneousCurvature*linearSpeed;
//  clampedCommand.instantaneousCurvature=instantaneousCurvature;
//  return clampedCommand;

//}


}//end romea
