// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include <romea_core_common/math/Algorithm.hpp>

// std
#include <cmath>
#include <limits>
#include <algorithm>

// local
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"

namespace romea
{

//--------------------------------------------------------------------------
SkidSteeringKinematic::Parameters::Parameters()
: wheelTrack(0),
  maximalWheelLinearSpeed(std::numeric_limits<double>::max()),
  maximalWheelLinearAcceleration(std::numeric_limits<double>::max()),
  wheelLinearSpeedVariance(0)
{
}

//--------------------------------------------------------------------------
double SkidSteeringKinematic::computeLinearSpeed(
  const double & leftWheelLinearSpeed,
  const double & rightWheelLinearSpeed)
{
  return 0.5 * (rightWheelLinearSpeed + leftWheelLinearSpeed);
}

//--------------------------------------------------------------------------
double SkidSteeringKinematic::computeAngularSpeed(
  const double & leftWheelLinearSpeed,
  const double & rightWheelLinearSpeed,
  const double & track)
{
  return (rightWheelLinearSpeed - leftWheelLinearSpeed) / track;
}

//--------------------------------------------------------------------------
double SkidSteeringKinematic::computeInstantaneousCurvature(
  const double & leftWheelLinearSpeed,
  const double & rightWheelLinearSpeed,
  const double & track)
{
  if (std::abs(leftWheelLinearSpeed) < std::numeric_limits<double>::epsilon() &&
    std::abs(rightWheelLinearSpeed) < std::numeric_limits<double>::epsilon())
  {
    return 0;
  }

  double v = computeLinearSpeed(leftWheelLinearSpeed, rightWheelLinearSpeed);
  double w = computeAngularSpeed(leftWheelLinearSpeed, rightWheelLinearSpeed, track);
  if (std::abs(v) < std::numeric_limits<double>::epsilon()) {
    return sign(w) * std::numeric_limits<double>::max();
  } else {
    return w / v;
  }
}

//--------------------------------------------------------------------------
double SkidSteeringKinematic::computeLeftWheelLinearSpeed(
  const double & linearSpeed,
  const double & angularSpeed,
  const double & wheelTrack)
{
  return linearSpeed - angularSpeed * wheelTrack / 2;
}

//--------------------------------------------------------------------------
double SkidSteeringKinematic::computeRightWheelLinearSpeed(
  const double & linearSpeed,
  const double & angularSpeed,
  const double & wheelTrack)
{
  return linearSpeed + angularSpeed * wheelTrack / 2;
}


//--------------------------------------------------------------------------
double SkidSteeringKinematic::minWheelLinearSpeed(
  const double & frontWheelLinearSpeed,
  const double & rearWheelLinearSpeed)
{
  if (frontWheelLinearSpeed > 0 && rearWheelLinearSpeed > 0) {
    return std::min(frontWheelLinearSpeed, rearWheelLinearSpeed);
  } else if (frontWheelLinearSpeed < 0 && rearWheelLinearSpeed < 0) {
    return std::max(frontWheelLinearSpeed, rearWheelLinearSpeed);
  } else {
    return 0;
  }
}

//--------------------------------------------------------------------------
SkidSteeringCommand clamp(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommandLimits & userLimits,
  const SkidSteeringCommand & command)
{
  // Clamp angular speed
  double maximalAbsoluteAngularSpeed =
    2 * parameters.maximalWheelLinearSpeed / parameters.wheelTrack;

  maximalAbsoluteAngularSpeed =
    std::min(maximalAbsoluteAngularSpeed, userLimits.angularSpeed.upper());

  double angularSpeed = romea::clamp(
    command.angularSpeed,
    -maximalAbsoluteAngularSpeed,
    maximalAbsoluteAngularSpeed);

  // Clamp linear speed
  double maximalAbsoluteLinearSpeed =
    parameters.maximalWheelLinearSpeed - std::abs(angularSpeed) * parameters.wheelTrack / 2.0;

  double minimalLinearSpeed =
    std::max(-maximalAbsoluteLinearSpeed, userLimits.longitudinalSpeed.lower());

  double maximalLinearSpeed =
    std::min(maximalAbsoluteLinearSpeed, userLimits.longitudinalSpeed.upper());

  double linearSpeed = romea::clamp(
    command.longitudinalSpeed,
    minimalLinearSpeed,
    maximalLinearSpeed);
  // clamp
  SkidSteeringCommand clampedCommand;
  clampedCommand.longitudinalSpeed = linearSpeed;
  clampedCommand.angularSpeed = angularSpeed;
  return clampedCommand;
}


//--------------------------------------------------------------------------
SkidSteeringCommand clamp(
  const SkidSteeringKinematic::Parameters & parameters,
  const SkidSteeringCommand & previousCommand,
  const SkidSteeringCommand & currentCommand,
  const double & dt)
{
  // Clamp angular speed
  double maximalAbsoluteAngularAcceleration =
    2 * parameters.maximalWheelLinearAcceleration / parameters.wheelTrack;

  double angularAccelaration = currentCommand.angularSpeed - previousCommand.angularSpeed;

  angularAccelaration = romea::clamp(
    angularAccelaration,
    -maximalAbsoluteAngularAcceleration,
    maximalAbsoluteAngularAcceleration);


  // Clamp linear speed
  double maximalAbsoluteLinearAcceleration = parameters.maximalWheelLinearAcceleration -
    std::abs(angularAccelaration) * parameters.wheelTrack / 2.0;

  double linearAcceleration = currentCommand.longitudinalSpeed - previousCommand.longitudinalSpeed;

  linearAcceleration = romea::clamp(
    linearAcceleration,
    -maximalAbsoluteLinearAcceleration,
    maximalAbsoluteLinearAcceleration);

  // return  command
  SkidSteeringCommand clampedCommand;
  clampedCommand.longitudinalSpeed = previousCommand.longitudinalSpeed + linearAcceleration * dt;
  clampedCommand.angularSpeed = previousCommand.angularSpeed + angularAccelaration * dt;
  return clampedCommand;
}

}  // namespace romea


// old codes
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
//                                                        const KinematicConstraints &userLimits)const
//{

//  assert(userLimits.isValidInstantaneousCurvature(instantaneousCurvature));

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
//                                                                   const KinematicConstraints &userLimits)const
//{
//  assert(userLimits.isValidSpeed(linearSpeed));

//  double absoluteLinearSpeed = std::abs(linearSpeed);
//  if(absoluteLinearSpeed < std::numeric_limits<float>::epsilon())
//    return userLimits.getMaximalAbsoluteInstantaneousCurvature();

//  //Compute maximal instantaneous curvature
//  double maximalIntantaneousCurvature = 2*(maximalWheelSpeed_/absoluteLinearSpeed-1)/getTrack("track");
//  maximalIntantaneousCurvature = std::min(maximalIntantaneousCurvature,userLimits.getMaximalAbsoluteInstantaneousCurvature());

//  //Saturation according maximal angular speed
//  double maximalAngularSpeed = computeMaximalAngularSpeed();
//  if(maximalAngularSpeed< maximalIntantaneousCurvature * absoluteLinearSpeed)
//    maximalIntantaneousCurvature = maximalAngularSpeed/absoluteLinearSpeed;

//  return maximalIntantaneousCurvature;
//}
