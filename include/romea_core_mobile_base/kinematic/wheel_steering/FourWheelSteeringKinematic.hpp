#ifndef romea_FourWheelSteeringKinematic_hpp
#define romea_FourWheelSteeringKinematic_hpp

//romea
#include "../axle_steering/TwoAxleSteeringKinematic.hpp"

namespace romea {

struct FourWheelSteeringKinematic
{

  struct Parameters
  {
    Parameters();
    double frontWheelBase;
    double rearWheelBase;
    double wheelTrack;
    double hubCarrierOffset;
    double maximalWheelSpeed;
    double maximalWheelAcceleration;
    double maximalWheelAngle;
    double maximalWheelAngularSpeed;
    double wheelSpeedVariance;
    double wheelAngleVariance;
  };


  static double comptuteBeta(const double & linearSpeedXBodyAxis,
                             const double & linearSpeedYBodyAxis);

  static double comptuteOrthogonalInstantaneousCurvature(const double & instantaneousCurvature,
                                                         const double & beta);

  static double computeFrontSteeringAngle(const double & instantaneousCurvature,
                                          const double & frontWheelBase,
                                          const double & beta);

  static double computeRearSteeringAngle(const double & instantaneousCurvature,
                                         const double & rearWheelBase,
                                         const double & beta);

};

TwoAxleSteeringCommand clamp(const FourWheelSteeringKinematic::Parameters & parameters,
                             const TwoAxleSteeringCommandLimits & userLimits,
                             const TwoAxleSteeringCommand & command);

TwoAxleSteeringCommand clamp(const FourWheelSteeringKinematic::Parameters & parameters,
                             const TwoAxleSteeringCommand & previousCommand,
                             const TwoAxleSteeringCommand &curentCommand,
                             const double & dt);

}
#endif
