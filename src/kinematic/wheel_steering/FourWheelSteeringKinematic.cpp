// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// romea core
#include <romea_core_common/math/Algorithm.hpp>

// std
#include <cmath>
#include <limits>

// local
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"

namespace romea
{
namespace core
{

//--------------------------------------------------------------------------
FourWheelSteeringKinematic::Parameters::Parameters()
: frontWheelBase(0),
  rearWheelBase(0),
  wheelTrack(0),
  hubCarrierOffset(0),
  maximalWheelLinearSpeed(std::numeric_limits<double>::max()),
  maximalWheelLinearAcceleration(std::numeric_limits<double>::max()),
  maximalWheelSteeringAngle(M_PI_2),
  maximalWheelSteeringAngularSpeed(std::numeric_limits<double>::max()),
  wheelLinearSpeedVariance(0),
  wheelSteeringAngleVariance(0)
{
}


//--------------------------------------------------------------------------
double FourWheelSteeringKinematic::comptuteBeta(
  const double & linearSpeedXBodyAxis,
  const double & linearSpeedYBodyAxis)
{
  return std::atan2(linearSpeedYBodyAxis, std::abs(linearSpeedXBodyAxis));
}


//--------------------------------------------------------------------------
double FourWheelSteeringKinematic::comptuteOrthogonalInstantaneousCurvature(
  const double & instantaneousCurvature,
  const double & beta)
{
  return instantaneousCurvature / std::cos(beta);
}

//--------------------------------------------------------------------------
double FourWheelSteeringKinematic::computeFrontSteeringAngle(
  const double & instantaneousCurvature,
  const double & frontWheelBase,
  const double & beta)
{
  return std::atan2(instantaneousCurvature * frontWheelBase + std::sin(beta), std::cos(beta));
}

//--------------------------------------------------------------------------
double FourWheelSteeringKinematic::computeRearSteeringAngle(
  const double & instantaneousCurvature,
  const double & rearWheelBase,
  const double & beta)
{
  return std::atan2(-instantaneousCurvature * rearWheelBase + std::sin(beta), std::cos(beta));
}

//--------------------------------------------------------------------------
TwoAxleSteeringCommand clamp(
  const FourWheelSteeringKinematic::Parameters & parameters,
  const TwoAxleSteeringCommandLimits & userLimits,
  const TwoAxleSteeringCommand & command)
{
  return TwoAxleSteeringKinematic::clamp(
    parameters.frontWheelBase,
    parameters.rearWheelBase,
    parameters.wheelTrack / 2.,
    parameters.wheelTrack / 2.,
    parameters.hubCarrierOffset,
    parameters.hubCarrierOffset,
    parameters.maximalWheelLinearSpeed,
    parameters.maximalWheelLinearSpeed,
    parameters.maximalWheelSteeringAngle,
    parameters.maximalWheelSteeringAngle,
    userLimits,
    command);
}

//--------------------------------------------------------------------------
TwoAxleSteeringCommand clamp(
  const FourWheelSteeringKinematic::Parameters & parameters,
  const TwoAxleSteeringCommand & previousCommand,
  const TwoAxleSteeringCommand & curentCommand,
  const double & dt)
{
  double maximalSteeringAngularSpeed = 0;

  return TwoAxleSteeringKinematic::clamp(
    parameters.frontWheelBase,
    parameters.rearWheelBase,
    parameters.wheelTrack / 2.,
    parameters.wheelTrack / 2.,
    parameters.hubCarrierOffset,
    parameters.hubCarrierOffset,
    parameters.maximalWheelLinearAcceleration,
    maximalSteeringAngularSpeed,
    previousCommand,
    curentCommand,
    dt);
}


}  // namespace core
}  // namespace romea


// old codes
// //--------------------------------------------------------------------------
// double maximalPermissibleLinearSpeed(
//   const FourWheelSteeringKinematic::Parameters & parameters,
//   const double & instantaneousCurvature)
// {

// }

// //--------------------------------------------------------------------------
// double maximalPermissibleInstantaneousCurvature(
//   const FourWheelSteeringKinematic::Parameters & parameters,
//   const double & linearSpeed)
// {

// }


// //--------------------------------------------------------------------------
// double FourWheelSteeringKinematic::computeMaximalInstantaneousCurvature()const
// {
//   double wheelBase = getWheelBase("wheelbase");
//   const double frontTrack = getTrack("front_track");
//   const double rearTrack = getTrack("rear_track");
//   const double track = (frontTrack + rearTrack) / 2.;

//   double maximalAbsoluteIntantaneousCurvature = OneAxleSteeringKinematic::
//     computeInstantaneousCurvature(maximalWheelAngle_, wheelBase / 2.0);

//   maximalAbsoluteIntantaneousCurvature = maximalAbsoluteIntantaneousCurvature /
//     (1 + maximalAbsoluteIntantaneousCurvature * track / 2.0);

//   return maximalAbsoluteIntantaneousCurvature;

// }

// //--------------------------------------------------------------------------
// double FourWheelSteeringKinematic::computeMaximalSteeringAngle()const
// {
//   double wheelBase = getWheelBase("wheelbase");
//   return OneAxleSteeringKinematic::computeSteeringAngle(
//     computeMaximalInstantaneousCurvature(), wheelBase / 2.0);
// }

// //--------------------------------------------------------------------------
// KinematicCommand FourWheelSteeringKinematic::clamp(
//   const KinematicCommand & command,
//   const KinematicConstraints & userLimits)const
// {


//   //clamp lateral speed
//   //double maximalAbsoluteBeta = maximalWheelAngle_;

//   double maximalAbsoluteLateralSpeed = maximalWheelSpeed_;
//   maximalAbsoluteLateralSpeed = std::min(
//     maximalAbsoluteLateralSpeed,
//     userLimits.getMaximalAbsoluteLinearSpeedAlongYBodyAxis());

//   double lateralSpeed = romea::clamp(
//     command.getLinearSpeedAlongYBodyAxis(),
//     -maximalAbsoluteLateralSpeed,
//     maximalAbsoluteLateralSpeed);

//   //clamp curvature
//   double cosBeta =
//     std::cos(std::atan(lateralSpeed / std::abs(command.getLinearSpeedAlongXBodyAxis())));

//   double maximalAbsoluteInstantaneousCurvature = 2 * cosBeta / track_;
//   maximalAbsoluteInstantaneousCurvature = std::min(
//     maximalAbsoluteInstantaneousCurvature,
//     userLimits.getMaximalAbsoluteInstantaneousCurvature());

//   double instantaneousCurvature = romea::clamp(
//     command.getInstantaneousCurvature(),
//     -maximalAbsoluteInstantaneousCurvature,
//     maximalAbsoluteInstantaneousCurvature);


//   //clamp linear speed

//   double maximalAbsoluteLinearSpeed = maximalWheelSpeed_ * std::cos(wheelAngle) /
//     (1 + std::abs(instantaneousCurvature) * track / (2 * std::cos(beta)));

// double absoluteMaximalAngularSpeed = userLimits.getMaximalAbsoluteAngularSpeedAroundZBodyAxis();
// if (absoluteMaximalAngularSpeed < std::abs(instantaneousCurvature) * maximalAbsoluteLinearSpeed) {  //NOLINT
//   maximalAbsoluteLinearSpeed = absoluteMaximalAngularSpeed / std::abs(instantaneousCurvature);
//   }

//   double minimalLinearSpeed = std::min(
//     -maximalAbsoluteLinearSpeed,
//     userLimits.getMinimalLinearSpeedAlongXBodyAxis());

//   double maximalLinearSpeed = std::min(
//     maximalAbsoluteLinearSpeed,
//     userLimits.getMaximalLinearSpeedAlongXBodyAxis());


//   double linearSpeed = romea::clamp(
//     command.getLinearSpeedAlongXBodyAxis(),
//     minimalLinearSpeed,
//     maximalLinearSpeed);


//   //make command
//   return makeFourWheelSteeringCommand(
//     command.getDuration(), linearSpeed, lateralSpeed, instantaneousCurvature);
// }
