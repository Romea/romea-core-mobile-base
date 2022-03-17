#ifndef romea_TwoWheelSteeringKinematic_hpp
#define romea_TwoWheelSteeringKinematic_hpp

//romea
#include "../axle_steering/OneAxleSteeringKinematic.hpp"
#include "../axle_steering/OneAxleSteeringCommand.hpp"

namespace romea {

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
    double frontMaximalWheelSpeed;
    double rearMaximalWheelSpeed;
    double maximalWheelAcceleration;
    double maximalWheelAngle;
    double maximalWheelAngularSpeed;
    double wheelSpeedVariance;
    double wheelAngleVariance;
  };


  static double computeInstantaneousCurvature(const double & leftInstantaneousCurvature,
                                              const double & rightInstantneousCurvature,
                                              const double & track);

  static double computeInstantaneousCurvature(const double & leftWheelAngle,
                                              const double & rightWheelAngle,
                                              const double & wheelbase,
                                              const double & track);

  static double computeSteeringAngle(const double & leftWheelAngle,
                                     const double & rightWheelAngle,
                                     const double & wheelbase,
                                     const double & track);


  static double computeMaximalInstantaneousCurvature(const double wheelbase,
                                                     const double halfTrack,
                                                     const double & maximalWheelAngle);


  static double computeLeftWheelAngle(const double & tanSteeringAngle,
                                      const double & instantaneousCurvature,
                                      const double & halfTrack);

  static double computeRightWheelAngle(const double & tanSteeringAngle,
                                       const double & instantaneousCurvature,
                                       const double & halfTrack);


};

OneAxleSteeringCommand clamp(const TwoWheelSteeringKinematic::Parameters & parameters,
                             const OneAxleSteeringCommandLimits & userLimits,
                             const OneAxleSteeringCommand & command);

OneAxleSteeringCommand clamp(const TwoWheelSteeringKinematic::Parameters & parameters,
                             const OneAxleSteeringCommand & previousCommand,
                             const OneAxleSteeringCommand & currentCommand,
                             const double & dt);

}
#endif
