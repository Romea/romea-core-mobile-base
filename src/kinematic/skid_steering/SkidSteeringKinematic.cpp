//romea
#include "romea_odo/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include <romea_common/math/Algorithm.hpp>

//std
#include <cmath>
#include <limits>
#include <algorithm>

namespace romea {

//--------------------------------------------------------------------------
SkidSteeringKinematic::Parameters::Parameters():
  track(0),
  maximalWheelSpeed(std::numeric_limits<double>::max()),
  maximalWheelAcceleration(std::numeric_limits<double>::max()),
  wheelSpeedVariance(0)
{

}

//--------------------------------------------------------------------------
double SkidSteeringKinematic::computeLinearSpeed(const double &leftWheelSpeed,
                                                 const double &rightWheelSpeed)
{
  return 0.5*(rightWheelSpeed+leftWheelSpeed);
}

//--------------------------------------------------------------------------
double SkidSteeringKinematic::computeAngularSpeed(const double &leftWheelSpeed,
                                                  const double &rightWheelSpeed,
                                                  const double &track)
{
  return (rightWheelSpeed-leftWheelSpeed)/track;
}

//--------------------------------------------------------------------------
double SkidSteeringKinematic::computeInstantaneousCurvature(const double &leftWheelSpeed,
                                                            const double & rightWheelSpeed,
                                                            const double & track)
{
  if(std::abs(leftWheelSpeed)<std::numeric_limits<double>::epsilon() &&
     std::abs(rightWheelSpeed)<std::numeric_limits<double>::epsilon())
  {
    return 0;
  }
  else
  {
    double v=computeLinearSpeed(leftWheelSpeed,rightWheelSpeed);
    double w = computeAngularSpeed(leftWheelSpeed,rightWheelSpeed,track);
    if(std::abs(v)<std::numeric_limits<double>::epsilon())
    {
      return sign(w)*std::numeric_limits<double>::max();
    }
    else
    {
      return w/v;
    }
  }
}

//--------------------------------------------------------------------------
double SkidSteeringKinematic::computeLeftWheelSpeed(const double & linearSpeed,
                                                    const double & angularSpeed,
                                                    const double & track)
{
  return linearSpeed-angularSpeed*track/2;
}

//--------------------------------------------------------------------------
double SkidSteeringKinematic::computeRightWheelSpeed(const double & linearSpeed,
                                                     const double & angularSpeed,
                                                     const double & track)
{
  return linearSpeed+angularSpeed*track/2;
}

////--------------------------------------------------------------------------
//double SkidSteeringKinematic::maximalPermissibleLinearSpeed(const double & track,
//                                                            const double & maximalWheelSpeed,
//                                                            const double & instantaneousCurvature)
//{
//  double absoluteInstantaneousCurvature = std::abs(instantaneousCurvature);
//  return maximalWheelSpeed/(1 + absoluteInstantaneousCurvature * track/2.0);
//}

////--------------------------------------------------------------------------
//double SkidSteeringKinematic::maximalPermissibleInstantaneousCurvature(const double & track,
//                                                                       const double & maximalWheelSpeed,
//                                                                       const double & linearSpeed)
//{
//  double absoluteLinearSpeed = std::abs(linearSpeed);
//  assert(absoluteLinearSpeed <=maximalWheelSpeed);

//  if(absoluteLinearSpeed < std::numeric_limits<float>::epsilon())
//  {
//    return std::numeric_limits<double>::max();
//  }
//  else
//  {
//    return 2*(maximalWheelSpeed/absoluteLinearSpeed-1)/track;
//  }
//}



////--------------------------------------------------------------------------
//double SkidSteeringKinematic::computeMaximalLinearSpeed(const double & instantaneousCurvature,
//                                                        const KinematicConstraints &userConstraints)const
//{

//  assert(userConstraints.isValidInstantaneousCurvature(instantaneousCurvature));

//  double absoluteInstantaneousCurvature = std::abs(instantaneousCurvature);
//  if( absoluteInstantaneousCurvature < std::numeric_limits<float>::epsilon() )
//    return maximalWheelSpeed_;

//  //Compute maximal speed
//  double maximalLinearSpeed = maximalWheelSpeed_/(1 + absoluteInstantaneousCurvature * getTrack("track")/2.0);

//  //Saturation according maximal angular speed
//  double maximalAngularSpeed = computeMaximalAngularSpeed();
//  if(maximalAngularSpeed< absoluteInstantaneousCurvature * maximalLinearSpeed)
//    maximalLinearSpeed = maximalAngularSpeed/absoluteInstantaneousCurvature;

//  return maximalLinearSpeed;
//}

////--------------------------------------------------------------------------
//double SkidSteeringKinematic::computeMaximalInstantaneousCurvature(const double & linearSpeed,
//                                                                   const KinematicConstraints &userConstraints)const
//{
//  assert(userConstraints.isValidSpeed(linearSpeed));

//  double absoluteLinearSpeed = std::abs(linearSpeed);
//  if(absoluteLinearSpeed < std::numeric_limits<float>::epsilon())
//    return userConstraints.getMaximalAbsoluteInstantaneousCurvature();

//  //Compute maximal instantaneous curvature
//  double maximalIntantaneousCurvature = 2*(maximalWheelSpeed_/absoluteLinearSpeed-1)/getTrack("track");
//  maximalIntantaneousCurvature = std::min(maximalIntantaneousCurvature,userConstraints.getMaximalAbsoluteInstantaneousCurvature());

//  //Saturation according maximal angular speed
//  double maximalAngularSpeed = computeMaximalAngularSpeed();
//  if(maximalAngularSpeed< maximalIntantaneousCurvature * absoluteLinearSpeed)
//    maximalIntantaneousCurvature = maximalAngularSpeed/absoluteLinearSpeed;

//  return maximalIntantaneousCurvature;
//}

//--------------------------------------------------------------------------
double SkidSteeringKinematic::minWheelSpeed(const double &frontWheelSpeed,
                                            const double & rearWheelSpeed)
{
  if(frontWheelSpeed > 0 && rearWheelSpeed > 0)
    return std::min(frontWheelSpeed,rearWheelSpeed);
  else if (frontWheelSpeed < 0 && rearWheelSpeed < 0)
    return std::max(frontWheelSpeed,rearWheelSpeed);
  else return 0;
}

//--------------------------------------------------------------------------
SkidSteeringCommand clamp(const SkidSteeringKinematic::Parameters &parameters,
                          const SkidSteeringConstraints & userConstraints,
                          const SkidSteeringCommand & command)
{
  //Clamp angular speed
  double maximalAbsoluteAngularSpeed = 2*parameters.maximalWheelSpeed/parameters.track;

  maximalAbsoluteAngularSpeed = std::min(maximalAbsoluteAngularSpeed,
                                         userConstraints.getMaximalAbsoluteAngularSpeed());

  double angularSpeed = romea::clamp(command.angularSpeed,
                                     -maximalAbsoluteAngularSpeed,
                                     maximalAbsoluteAngularSpeed);


  //Clamp linear speed
  double maximalAbsoluteLinearSpeed= parameters.maximalWheelSpeed - std::abs(angularSpeed)* parameters.track/2.0;

  double minimalLinearSpeed = std::max(-maximalAbsoluteLinearSpeed,
                                       userConstraints.getMinimalLinearSpeed());

  double maximalLinearSpeed = std::min(maximalAbsoluteLinearSpeed,
                                       userConstraints.getMaximalLinearSpeed());


  double linearSpeed = romea::clamp(command.longitudinalSpeed,
                                    minimalLinearSpeed,
                                    maximalLinearSpeed);
  //clamp
  SkidSteeringCommand clampedCommand;
  clampedCommand.longitudinalSpeed=linearSpeed;
  clampedCommand.angularSpeed=angularSpeed;
  return clampedCommand;

}

////--------------------------------------------------------------------------
//double maximalPermissibleLinearSpeed(const SkidSteeringKinematic::Parameters & parameters,
//                                     const double & instantaneousCurvature)
//{
//  return SkidSteeringKinematic::
//      maximalPermissibleLinearSpeed(parameters.track,
//                                    parameters.maximalWheelSpeed,
//                                    instantaneousCurvature);

//}

////--------------------------------------------------------------------------
//double maximalPermissibleInstantaneousCurvature(const SkidSteeringKinematic::Parameters & parameters,
//                                                const double & linearSpeed)
//{
//  return SkidSteeringKinematic::
//      maximalPermissibleInstantaneousCurvature(parameters.track,
//                                               parameters.maximalWheelSpeed,
//                                               linearSpeed);

//}




}//end romea
