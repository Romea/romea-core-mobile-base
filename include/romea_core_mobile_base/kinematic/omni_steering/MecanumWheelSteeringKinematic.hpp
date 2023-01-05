// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__MECANUMWHEELSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__MECANUMWHEELSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommandLimits.hpp"

namespace romea
{

struct MecanumWheelSteeringKinematic
{
  struct Parameters
  {
    Parameters();
    double wheelTrack;
    double wheelbase;
    double maximalWheelLinearSpeed;
    double maximalWheelLinearAcceleration;
    double wheelLinearSpeedVariance;
  };


  static double computeFrontLeftWheelLinearSpeed(
    const double & longitidutinalSpeed,
    const double & lateralSpeed,
    const double & angularSpeed,
    const double & halfWheelbase,
    const double & halfTrack);

  static double computeFrontRightWheelLinearSpeed(
    const double & longitidutinalSpeed,
    const double & lateralSpeed,
    const double & angularSpeed,
    const double & halfWheelbase,
    const double & halfTrack);

  static double computeRearLeftWheelLinearSpeed(
    const double & longitidutinalSpeed,
    const double & lateralSpeed,
    const double & angularSpeed,
    const double & halfWheelbase,
    const double & halfTrack);

  static double computeRearRightWheelLinearSpeed(
    const double & longitidutinalSpeed,
    const double & lateralSpeed,
    const double & angularSpeed,
    const double & halfWheelbase,
    const double & halfTrack);


  static double computeLongitudinalSpeed(
    const double & frontLeftWheelSpeed,
    const double & frontRightWheelSpeed,
    const double & rearLeftWheelSpeed,
    const double & rearRightWheelSpeed);

  static double computeLateralSpeed(
    const double & frontLeftWheelSpeed,
    const double & frontRightWheelSpeed,
    const double & rearLeftWheelSpeed,
    const double & rearRightWheelSpeed);


  static double computeAngularSpeed(
    const double & frontLeftWheelSpeed,
    const double & frontRightWheelSpeed,
    const double & rearLeftWheelSpeed,
    const double & rearRightWheelSpeed,
    const double & halfWheelbase,
    const double & halfTrack);
};

OmniSteeringCommand clamp(
  const MecanumWheelSteeringKinematic::Parameters & parameters,
  const OmniSteeringCommandLimits & userLimits,
  const OmniSteeringCommand & command);

OmniSteeringCommand clamp(
  const MecanumWheelSteeringKinematic::Parameters & parameters,
  const OmniSteeringCommand & previousCommand,
  const OmniSteeringCommand & currentCommand,
  const double & dt);


}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE__KINEMATIC__OMNI_STEERING__MECANUMWHEELSTEERINGKINEMATIC_HPP_
