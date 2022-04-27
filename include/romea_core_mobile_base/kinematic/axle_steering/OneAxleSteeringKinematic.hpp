#ifndef romea_OneAxleSteeringKinematic_hpp
#define romea_OneAxleSteeringKinematic_hpp

//romea
#include "OneAxleSteeringCommand.hpp"
#include "OneAxleSteeringCommandLimits.hpp"

//std
#include <cmath>

namespace romea {

struct OneAxleSteeringKinematic
{

  struct Parameters
  {
    Parameters();
    double frontWheelBase;
    double rearWheelBase;
    double frontWheelTrack;
    double rearWheelTrack;
    double frontHubCarrierOffset;
    double rearHubCarrierOffset;
    double frontMaximalWheelSpeed;
    double rearMaximalWheelSpeed;
    double maximalWheelAcceleration;
    double maximalSteeringAngle;
    double maximalSteeringAngularSpeed;
    double wheelSpeedVariance;
    double steeringAngleVariance;
  };


  static double computeBeta(const double & tanSteeringAngle,
                            const double & frontWheelBase,
                            const double & rearWheelBase);

  static double computeInstantaneousCurvature(const double & tanSteeringAngle,
                                              const double & wheelBase);


  static double computeSteeringAngle(const double &instantaneousCurvature,
                                     const double &wheelBase );


  static double computeAngularSpeed(const double & linearSpeed,
                                    const double & instantaneousCurvature);


  static double computeWheelSpeedRatio(const double & tanSteeringAngle,
                                       const double & instaneousCurvature,
                                       const double & hubCarrierOffset,
                                       const double & halfTrack);

  static double computeLeftWheelSpeed(const double & linearSpeed,
                                      const double & tanSteeringAngle,
                                      const double & instaneousCurvature,
                                      const double & hubCarrierOffset,
                                      const double & halfTrack);

  static double computeRightWheelSpeed(const double & linearSpeed,
                                       const double & tanSteeringAngle,
                                       const double & instaneousCurvature,
                                       const double & hubCarrierOffset,
                                       const double & halfTrack);

  static double computeLinearSpeed(const double & leftWheelSpeed,
                                   const double & rightWheelSpeed,
                                   const double & tanSteeringAngle,
                                   const double & instaneousCurvature,
                                   const double & hubCarrierOffset,
                                   const double & halfTrack);

  static OneAxleSteeringCommand clamp(const double & wheelbase,
                                      const double & frontHalfTrack,
                                      const double & rearHalfTrack,
                                      const double & frontHubCarrierOffset,
                                      const double & rearHubCarrierOffset,
                                      const double & maximalSteeringAngle,
                                      const double & frontMaximalWheelSpeed,
                                      const double & rearMaximalWheelSpeed,
                                      const OneAxleSteeringCommandLimits & userLimits,
                                      const OneAxleSteeringCommand & command);

  static OneAxleSteeringCommand clamp(const double & wheelbase,
                                      const double & frontHalfTrack,
                                      const double & rearHalfTrack,
                                      const double & frontHubCarrierOffset,
                                      const double & rearHubCarrierOffset,
                                      const double & maximalSteeringAngularSpeed,
                                      const double & maximalWheelAcceleration,
                                      const OneAxleSteeringCommand & previousCommand,
                                      const OneAxleSteeringCommand & currentCommand,
                                      const double & dt);

};


OneAxleSteeringCommand clamp(const OneAxleSteeringKinematic::Parameters & parameters,
                             const OneAxleSteeringCommandLimits & userLimits,
                             const OneAxleSteeringCommand & command);

OneAxleSteeringCommand clamp(const OneAxleSteeringKinematic::Parameters & parameters,
                             const OneAxleSteeringCommand & previousCommand,
                             const OneAxleSteeringCommand & currentCommand,
                             const double & dt);

}
#endif
