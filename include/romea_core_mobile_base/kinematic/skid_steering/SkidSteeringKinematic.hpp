#ifndef ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_SKIDSTEERINGKINEMATIC_HPP_
#define ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_SKIDSTEERINGKINEMATIC_HPP_

// romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommandLimits.hpp"

namespace romea
{

struct SkidSteeringKinematic
{
  struct Parameters
  {
    Parameters();
    double wheelTrack;
    double maximalWheelLinearSpeed;
    double maximalWheelLinearAcceleration;
    double wheelLinearSpeedVariance;
  };

  static double computeLinearSpeed(const double &leftWheelLinearSpeed,
                                   const double &rightWheelLinearSpeed);

  static double computeAngularSpeed(const double &leftWheelLinearSpeed,
                                    const double &rightWheelLinearSpeed,
                                    const double & track);

  static double computeInstantaneousCurvature(const double &leftWheelLinearSpeed,
                                              const double & rightWheelLinearSpeed,
                                              const double & track);

  static double computeLeftWheelLinearSpeed(const double & linearSpeed ,
                                            const double & angularSpeed,
                                            const double & wheelTrack);

  static double computeRightWheelLinearSpeed(const double & linearSpeed ,
                                             const double & angularSpeed,
                                             const double & wheelTrack);

  static double minWheelLinearSpeed(const double &frontWheelLinearSpeed,
                                    const double & rearWheelLinearSpeed);
};

SkidSteeringCommand clamp(const SkidSteeringKinematic::Parameters & parameters,
                          const SkidSteeringCommandLimits & userLimits,
                          const SkidSteeringCommand & command);

SkidSteeringCommand clamp(const SkidSteeringKinematic::Parameters & parameters,
                          const SkidSteeringCommand & previousCommand,
                          const SkidSteeringCommand & currentCommand,
                          const double & dt);

}  // namespace romea

#endif  // ROMEA_CORE_MOBILE_BASE_KINEMATIC_SKID_STEERING_SKIDSTEERINGKINEMATIC_HPP_
