// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__TWOWHEELSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__TWOWHEELSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp"

namespace romea
{

class TwoWheelSteeringKinematic : public OneAxleSteeringKinematic
{
public:
  struct Parameters
  {
    Parameters();
    double frontWheelBase;
    double rearWheelBase;
    double frontWheelTrack;
    double rearWheelTrack;
    double frontHubCarrierOffset;
    double rearHubCarrierOffset;
    double frontMaximalWheelLinearSpeed;
    double rearMaximalWheelLinearSpeed;
    double maximalWheelLinearAcceleration;
    double maximalWheelSteeringAngle;
    double maximalWheelSteeringAngularSpeed;
    double wheelLinearSpeedVariance;
    double wheelSteeringAngleVariance;
  };


  static double computeInstantaneousCurvature(
    const double & leftInstantaneousCurvature,
    const double & rightInstantneousCurvature,
    const double & track);

  static double computeInstantaneousCurvature(
    const double & leftWheelSteeringAngle,
    const double & rightWheelSteeringAngle,
    const double & wheelbase,
    const double & track);

  static double computeSteeringAngle(
    const double & leftWheelSteeringAngle,
    const double & rightWheelSteeringAngle,
    const double & wheelbase,
    const double & track);


  static double computeMaximalInstantaneousCurvature(
    const double wheelbase,
    const double halfTrack,
    const double & maximalWheelSteeringAngle);


  static double computeLeftWheelSteeringAngle(
    const double & tanSteeringAngle,
    const double & instantaneousCurvature,
    const double & halfTrack);

  static double computeRightWheelSteeringAngle(
    const double & tanSteeringAngle,
    const double & instantaneousCurvature,
    const double & halfTrack);
};

OneAxleSteeringCommand clamp(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommandLimits & userLimits,
  const OneAxleSteeringCommand & command);

OneAxleSteeringCommand clamp(
  const TwoWheelSteeringKinematic::Parameters & parameters,
  const OneAxleSteeringCommand & previousCommand,
  const OneAxleSteeringCommand & currentCommand,
  const double & dt);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__WHEEL_STEERING__TWOWHEELSTEERINGKINEMATIC_HPP_
