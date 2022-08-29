//romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

//std
#include <cmath>
#include <limits>
#include <iostream>

namespace romea {

//--------------------------------------------------------------------------
OneAxleSteeringKinematic::Parameters::Parameters():
  frontWheelBase(0),
  rearWheelBase(0),
  frontWheelTrack(0),
  rearWheelTrack(0),
  frontHubCarrierOffset(0),
  rearHubCarrierOffset(0),
  frontMaximalWheelLinearSpeed(std::numeric_limits<double>::max()),
  rearMaximalWheelLinearSpeed(std::numeric_limits<double>::max()),
  maximalWheelLinearAcceleration(std::numeric_limits<double>::max()),
  maximalSteeringAngle(M_PI_2),
  maximalSteeringAngularSpeed(std::numeric_limits<double>::max()),
  wheelLinearSpeedVariance(0),
  steeringAngleVariance(0)

{

}

//--------------------------------------------------------------------------
double OneAxleSteeringKinematic::computeBeta(const double & tanSteeringAngle,
                                             const double & frontWheelBase,
                                             const double & rearWheelBase)
{
  double wheelbase= frontWheelBase+rearWheelBase;
  return std::atan(tanSteeringAngle*rearWheelBase/wheelbase);
}

//--------------------------------------------------------------------------
double OneAxleSteeringKinematic::computeAngularSpeed(const double & linearSpeed,
                                                     const double & instantaneousCurvature)
{
  return linearSpeed * instantaneousCurvature;
}

//--------------------------------------------------------------------------
double OneAxleSteeringKinematic::computeInstantaneousCurvature(const double & tanSteeringAngle,
                                                               const double & wheelBase)
{
  return tanSteeringAngle/wheelBase;
}

//--------------------------------------------------------------------------
double OneAxleSteeringKinematic::computeSteeringAngle(const double & instantaneousCurvature,
                                                      const double & wheelBase)
{
  return std::atan(instantaneousCurvature*wheelBase);
}


//--------------------------------------------------------------------------
double OneAxleSteeringKinematic::computeWheelLinearSpeedRatio(const double & tanSteeringAngle,
                                                              const double & instaneousCurvature,
                                                              const double & hubCarrierOffset,
                                                              const double & halfTrack)
{
  return std::sqrt(std::pow(1+instaneousCurvature*halfTrack,2.)+tanSteeringAngle*tanSteeringAngle)+instaneousCurvature*hubCarrierOffset;
}

//--------------------------------------------------------------------------
double OneAxleSteeringKinematic::computeLeftWheelLinearSpeed(const double & linearSpeed,
                                                             const double & tanSteeringAngle,
                                                             const double & instaneousCurvature,
                                                             const double & hubCarrierOffset,
                                                             const double & halfTrack)
{
  return linearSpeed*computeWheelLinearSpeedRatio(-tanSteeringAngle,-instaneousCurvature,hubCarrierOffset,halfTrack);
}

//--------------------------------------------------------------------------
double OneAxleSteeringKinematic::computeRightWheelLinearSpeed(const double & linearSpeed,
                                                              const double & tanSteeringAngle,
                                                              const double & instaneousCurvature,
                                                              const double & hubCarrierOffset,
                                                              const double & halfTrack)
{
  return linearSpeed*computeWheelLinearSpeedRatio(tanSteeringAngle,instaneousCurvature,hubCarrierOffset,halfTrack);
}

//--------------------------------------------------------------------------
double OneAxleSteeringKinematic::computeLinearSpeed(const double & leftWheelSpeed,
                                                    const double & rightWheelSpeed,
                                                    const double & tanSteeringAngle,
                                                    const double & instantaneousCurvature,
                                                    const double & hubCarrierOffset,
                                                    const double & halfTrack)
{
  double leftWheelSpeedRatio= computeWheelLinearSpeedRatio(-tanSteeringAngle,
                                                           -instantaneousCurvature,
                                                           hubCarrierOffset,
                                                           halfTrack);

  double rightWheelSpeedRatio= computeWheelLinearSpeedRatio(tanSteeringAngle,
                                                            instantaneousCurvature,
                                                            hubCarrierOffset,
                                                            halfTrack);


  return (leftWheelSpeed/leftWheelSpeedRatio+
          rightWheelSpeed/rightWheelSpeedRatio)/2.0;

}




//--------------------------------------------------------------------------
OneAxleSteeringCommand OneAxleSteeringKinematic::clamp(const double & wheelbase,
                                                       const double & frontHalfTrack,
                                                       const double & rearHalfTrack,
                                                       const double & frontHubCarrierOffset,
                                                       const double & rearHubCarrierOffset,
                                                       const double & maximalSteeringAngle,
                                                       const double & frontMaximalWheelSpeed,
                                                       const double & rearMaximalWheelSpeed,
                                                       const OneAxleSteeringCommandLimits & userLimits,
                                                       const OneAxleSteeringCommand & command)
{
  //clamp steering angle
  double maximalAbsoluteSteeringAngle = std::min(maximalSteeringAngle,userLimits.steeringAngle.upper());

  double steeringAngle =romea::clamp(command.steeringAngle,
                                     -maximalAbsoluteSteeringAngle,
                                     maximalAbsoluteSteeringAngle);


  //clamp linear speed
  double maximalAbsoluteLinearSpeed = std::numeric_limits<double>::max();
  double tanSteeringAngle = std::tan(steeringAngle);
  double instantaneousCurvature = tanSteeringAngle/wheelbase;

  double rearRatio = (1+std::abs(tanSteeringAngle)*(rearHalfTrack+2*rearHubCarrierOffset)/wheelbase);
  maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,rearMaximalWheelSpeed/rearRatio);

  double frontRatio = OneAxleSteeringKinematic::computeWheelLinearSpeedRatio(std::abs(tanSteeringAngle),std::abs(instantaneousCurvature),frontHubCarrierOffset,frontHalfTrack);
  maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,frontMaximalWheelSpeed/frontRatio);

  double minimalLinearSpeed = std::max(-maximalAbsoluteLinearSpeed,
                                       userLimits.longitudinalSpeed.lower());

  double maximalLinearSpeed = std::min(maximalAbsoluteLinearSpeed,
                                       userLimits.longitudinalSpeed.upper());

  double linearSpeed = romea::clamp(command.longitudinalSpeed,
                                    minimalLinearSpeed,
                                    maximalLinearSpeed);

  //return command
  OneAxleSteeringCommand clamped_command;
  clamped_command.longitudinalSpeed=linearSpeed;
  clamped_command.steeringAngle=steeringAngle;
  return clamped_command;
}

//--------------------------------------------------------------------------
OneAxleSteeringCommand OneAxleSteeringKinematic::clamp(const double & wheelbase,
                                                       const double & frontHalfTrack,
                                                       const double & rearHalfTrack,
                                                       const double & frontHubCarrierOffset,
                                                       const double & rearHubCarrierOffset,
                                                       const double & maximalSteeringAngularSpeed,
                                                       const double & maximalWheelAcceleration,
                                                       const OneAxleSteeringCommand & previousCommand,
                                                       const OneAxleSteeringCommand & currentCommand,
                                                       const double & dt)
{
  //clamp steering angular speed
  double steeringAngularSpeed =currentCommand.steeringAngle-previousCommand.steeringAngle;

  steeringAngularSpeed =romea::clamp(steeringAngularSpeed,
                                     -maximalSteeringAngularSpeed,
                                     maximalSteeringAngularSpeed);

  //clamp linear acceleration
  double maximalAbsoluteLinearAcceleration = std::numeric_limits<double>::max();
  double tanSteeringAngle = std::tan(previousCommand.steeringAngle);
  double instantaneousCurvature = previousCommand.steeringAngle/wheelbase;

  double rearRatio = (1+std::abs(tanSteeringAngle)*(rearHalfTrack+2*rearHubCarrierOffset)/wheelbase);
  maximalAbsoluteLinearAcceleration = std::min(maximalAbsoluteLinearAcceleration,maximalWheelAcceleration/rearRatio);

  double frontRatio = OneAxleSteeringKinematic::computeWheelLinearSpeedRatio(std::abs(tanSteeringAngle),std::abs(instantaneousCurvature),frontHubCarrierOffset,frontHalfTrack);
  maximalAbsoluteLinearAcceleration = std::min(maximalAbsoluteLinearAcceleration,maximalWheelAcceleration/frontRatio);

  double linearAcceleration = currentCommand.longitudinalSpeed-previousCommand.longitudinalSpeed;

  linearAcceleration = romea::clamp(linearAcceleration,
                                    -maximalAbsoluteLinearAcceleration,
                                    maximalAbsoluteLinearAcceleration);

  //return command
  OneAxleSteeringCommand clamped_command;
  clamped_command.longitudinalSpeed=previousCommand.longitudinalSpeed+linearAcceleration*dt;
  clamped_command.steeringAngle=previousCommand.steeringAngle+steeringAngularSpeed*dt;
  return clamped_command;

}

//--------------------------------------------------------------------------
OneAxleSteeringCommand clamp(const OneAxleSteeringKinematic::Parameters & parameters,
                             const OneAxleSteeringCommandLimits & userLimits,
                             const OneAxleSteeringCommand & command)
{
  return OneAxleSteeringKinematic::clamp(parameters.rearWheelBase+parameters.frontWheelBase,
                                         parameters.frontWheelTrack/2.,
                                         parameters.rearWheelTrack/2.,
                                         parameters.frontHubCarrierOffset,
                                         parameters.rearHubCarrierOffset,
                                         parameters.maximalSteeringAngle,
                                         parameters.frontMaximalWheelLinearSpeed,
                                         parameters.rearMaximalWheelLinearSpeed,
                                         userLimits,
                                         command);
}

//--------------------------------------------------------------------------
OneAxleSteeringCommand clamp(const OneAxleSteeringKinematic::Parameters & parameters,
                             const OneAxleSteeringCommand & previousCommand,
                             const OneAxleSteeringCommand & currentCommand,
                             const double & dt)
{
  return OneAxleSteeringKinematic::clamp(parameters.rearWheelBase+parameters.frontWheelBase,
                                         parameters.frontWheelTrack/2.,
                                         parameters.rearWheelTrack/2.,
                                         parameters.frontHubCarrierOffset,
                                         parameters.rearHubCarrierOffset,
                                         parameters.maximalSteeringAngularSpeed,
                                         parameters.maximalWheelLinearAcceleration,
                                         previousCommand,
                                         currentCommand,
                                         dt);
}



////--------------------------------------------------------------------------
//double OneAxleSteeringKinematic::maximalPermissibleLinearSpeed(const double & wheelbase,
//                                                               const double & track,
//                                                               const double & maximalWheelSpeed,
//                                                               const double & maximalSteeringAngle,
//                                                               const double & hubCarrierOffset,
//                                                               const double & instantaneousCurvature)
//{



//  double absoluteInstantaneousCurvature = std::abs(instantaneousCurvature);

//  assert(absoluteInstantaneousCurvature < std::tan(maximalSteeringAngle)/wheelbase);

//  return  maximalWheelSpeed/computeWheelSpeedRatio(absoluteInstantaneousCurvature*wheelbase,
//                                                   absoluteInstantaneousCurvature,
//                                                   hubCarrierOffset,
//                                                   track/2.);

//}

////--------------------------------------------------------------------------
//double OneAxleSteeringKinematic::maximalPermissibleInstantaneousCurvature(const double & wheelbase,
//                                                                          const double & track,
//                                                                          const double & maximalWheelSpeed,
//                                                                          const double & maximalSteeringAngle,
//                                                                          const double & hubCarrierOffset,
//                                                                          const double & linearSpeed)
//{
//  assert(linearSpeed<maximalWheelSpeed);

//  //For linearSpeed > 0 && instantaneousCurvature > 0
//  //Right wheel speed is higher than left wheel speed
//  //So we consider only right wheel

//  //We want to solve  vRightMax  - linearSpeed * (sqrt((1+KmaxT/2)^+(KmaxL)^2)+Kmaxh)  = 0 where :
//  //vRightMax = maximalWheelSpeed
//  //T = track
//  //L = wheelbase
//  //h = hubCarrierOffset
//  //Kmax =  maximal instantaneous curvature
//  //For this it means the following second order function

//  double absoluteLinearSpeed = std::abs(linearSpeed);
//  double a = (wheelbase+hubCarrierOffset)*(wheelbase+hubCarrierOffset) + track*track/4.;
//  double b = track;
//  double c = 1 - (maximalWheelSpeed*maximalWheelSpeed)/(absoluteLinearSpeed*absoluteLinearSpeed);
//  double d = b*b - 4*a*c;


//  return std::min((-b + std::sqrt(d))/(2*a), std::tan(maximalSteeringAngle)/wheelbase);


//}

////--------------------------------------------------------------------------
//double OneAxleSteeringKinematic::computeMaximalLinearSpeed(const double & instantaneousCurvature,
//                                                           const KinematicConstraints &userLimits)const
//{

//  assert(userLimits.isValidInstantaneousCurvature(instantaneousCurvature));

//  double absoluteInstantaneousCurvature = std::abs(instantaneousCurvature);
//  if( absoluteInstantaneousCurvature < std::numeric_limits<float>::epsilon() )
//    return maximalWheelSpeed_;

//  //For linearSpeed > 0 && instantaneousCurvature > 0
//  //Right wheel speed is higher than left wheel speed
//  //So we consider only right wheel

//  //We don't use this code
//  //double steeringAngle = computeSteeringAngle(absoluteInstantaneousCurvature);
//  //double rightWheelAngle = computeRightWheelAngle(absoluteInstantaneousCurvature);
//  //double maximalLinearSpeed = maximalWheelSpeed_ /std::tan(steeringAngle)*std::sin(rightWheelAngle);
//  //But this one in order to the exact inverse function of computeMaximalInstantaneousCurvatureAccordingLinearSpeed

//  double wheelBase = getWheelBase("wheelbase");
//  double track = getTrack("front_track");
//  double a = wheelBase*wheelBase + track+track/4.;
//  double b = track;
//  double c = 1;

//  double maximalLinearSpeed = maximalWheelSpeed_
//      /sqrt(a*absoluteInstantaneousCurvature*absoluteInstantaneousCurvature+
//            b*absoluteInstantaneousCurvature +c);

//  //saturation according maximal angular speed
//  double maximalAngularSpeed = userLimits.getMaximalAbsoluteAngularSpeed();
//  if(maximalAngularSpeed< absoluteInstantaneousCurvature * maximalLinearSpeed)
//    maximalLinearSpeed = maximalAngularSpeed/absoluteInstantaneousCurvature;

//  return maximalLinearSpeed;
//}

////--------------------------------------------------------------------------
//double OneAxleSteeringKinematic::computeMaximalInstantaneousCurvature(const double & linearSpeed,
//                                                                      const KinematicConstraints & userLimits)const
//{

//  double absoluteLinearSpeed = std::abs(linearSpeed);
//  if(absoluteLinearSpeed < std::numeric_limits<float>::epsilon())
//    return maximalWheelSpeed_;

//  //For linearSpeed > 0 && instantaneousCurvature > 0
//  //Right wheel speed is higher than left wheel speed
//  //So we consider only right wheel

//  //double steeringAngle = computeSteeringAngle(absoluteInstantaneousCurvature);
//  //double rightWheelAngle = computeRightWheelAngle(absoluteInstantaneousCurvature);
//  //We want to solve  vRightMax  - linearSpeed * tan(steeringAngle) / sin(rightWheelAngle)  = 0
//  //For this it means the following second order function

//  double wheelBase = getWheelBase("wheelBase");
//  double track = getTrack("front_track");
//  double a = wheelBase*wheelBase + track+track/4.;
//  double b = track;
//  double c = 1 - (maximalWheelSpeed_*maximalWheelSpeed_)/(absoluteLinearSpeed*absoluteLinearSpeed);
//  double d = b*b - 4*a*c;

//  double maximalInstantaneousCurvature = (-b + std::sqrt(d))/(2*a);
//  maximalInstantaneousCurvature = std::min(maximalInstantaneousCurvature,
//                                           userLimits.getMaximalAbsoluteInstantaneousCurvature());

//  //saturation according maximal angular speed
//  double maximalAngularSpeed = userLimits.getMaximalAbsoluteAngularSpeed();
//  if(maximalAngularSpeed< maximalInstantaneousCurvature * absoluteLinearSpeed)
//    maximalInstantaneousCurvature = maximalAngularSpeed/absoluteLinearSpeed;

//  return maximalInstantaneousCurvature;


//}

////--------------------------------------------------------------------------
//double maximalPermissibleLinearSpeed(const OneAxleSteeringKinematic::Parameters & parameters,
//                                     const double & instantaneousCurvature)
//{
//  double frontMaximalLinearSpeed= OneAxleSteeringKinematic::
//      maximalPermissibleLinearSpeed(parameters.rearWheelBase+parameters.frontWheelBase,
//                                    parameters.frontTrack,
//                                    parameters.frontMaximalWheelSpeed,
//                                    parameters.maximalSteeringAngle,
//                                    parameters.frontHubCarrierOffset,
//                                    instantaneousCurvature);

//  double rearMaximalLinearSpeed= SkidSteeringKinematic::
//      maximalPermissibleLinearSpeed(parameters.rearTrack+2*parameters.rearHubCarrierOffset,
//                                    parameters.rearMaximalWheelSpeed,
//                                    instantaneousCurvature);

//  return std::min(frontMaximalLinearSpeed,rearMaximalLinearSpeed);
//}

////--------------------------------------------------------------------------
//double maximalPermissibleInstantaneousCurvature(const OneAxleSteeringKinematic::Parameters & parameters,
//                                                const double & linearSpeed)
//{
//  double frontMaximalInstantaneousCurvature= OneAxleSteeringKinematic::
//      maximalPermissibleInstantaneousCurvature(parameters.rearWheelBase+parameters.frontWheelBase,
//                                               parameters.frontTrack,
//                                               parameters.frontMaximalWheelSpeed,
//                                               parameters.maximalSteeringAngle,
//                                               parameters.frontHubCarrierOffset,
//                                               linearSpeed);

//  double rearMaximalInstantaneousCurvature= SkidSteeringKinematic::
//      maximalPermissibleInstantaneousCurvature(parameters.rearTrack+2*parameters.rearHubCarrierOffset,
//                                               parameters.rearMaximalWheelSpeed,
//                                               linearSpeed);

//  return std::min(frontMaximalInstantaneousCurvature,rearMaximalInstantaneousCurvature);

//}






}//end romea
