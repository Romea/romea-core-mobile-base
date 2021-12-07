#ifndef romea_TwoAxleSteeringKinematic_hpp
#define romea_TwoAxleSteeringKinematic_hpp

//romea
#include "TwoAxleSteeringCommand.hpp"
#include "TwoAxleSteeringConstraints.hpp"

//std
#include <cmath>

namespace romea {

struct TwoAxleSteeringKinematic
{


  struct Parameters
  {
    Parameters();
    double frontWheelBase;
    double rearWheelBase;
    double frontTrack;
    double rearTrack;
    double frontHubCarrierOffset;
    double rearHubCarrierOffset;
    double frontMaximalWheelSpeed;
    double rearMaximalWheelSpeed;
    double maximalWheelAcceleration;
    double frontMaximalSteeringAngle;
    double rearMaximalSteeringAngle;
    double maximalSteeringAngularSpeed;
    double wheelSpeedVariance;
    double steeringAngleVariance;
  };


  static double computeBeta(const double & tanFrontSteeringAngle,
                            const double & tanRearSteeringAngle,
                            const double & frontWheelBase,
                            const double & rearWheelBase);

  static double computeOrthogonalInstantaneousCurvature(const double & tanFrontSteeringAngle,
                                                        const double & tanRearSteeringAngle,
                                                        const double & frontWheelBase,
                                                        const double & rearWheelBase);

  static double computeInstantaneousCurvature(const double & tanFrontSteeringAngle,
                                              const double & tanRearSteeringAngle,
                                              const double & frontWheelBase,
                                              const double & rearWheelBase);

  static TwoAxleSteeringCommand clamp(const double  & frontWheelBase,
                                      const double  & rearWheelBase,
                                      const double  & frontHalfTrack,
                                      const double  & rearHalfTrack,
                                      const double  & frontHubCarrierOffset,
                                      const double  & rearHubCarrierOffset,
                                      const double  & frontMaximalWheelSpeed,
                                      const double  & rearMaximalWheelSpeed,
                                      const double  & frontMaximalSteeringAngle,
                                      const double  & rearMaximalSteeringAngle,
                                      const TwoAxleSteeringConstraints & userConstraints,
                                      const TwoAxleSteeringCommand & command);

  static TwoAxleSteeringCommand clamp(const double  & frontWheelBase,
                                      const double  & rearWheelBase,
                                      const double  & frontHalfTrack,
                                      const double  & rearHalfTrack,
                                      const double  & frontHubCarrierOffset,
                                      const double  & rearHubCarrierOffset,
                                      const double  & maximalWheelAcceleration,
                                      const double  & maximalSteeringAngularSpeed,
                                      const TwoAxleSteeringCommand & previousCommand,
                                      const TwoAxleSteeringCommand & currentCommand,
                                      const double & dt);

};


TwoAxleSteeringCommand clamp(const TwoAxleSteeringKinematic::Parameters & parameters,
                             const TwoAxleSteeringConstraints & userConstraints,
                             const TwoAxleSteeringCommand & command);

TwoAxleSteeringCommand clamp(const TwoAxleSteeringKinematic::Parameters & parameters,
                             const TwoAxleSteeringCommand & previousCommand,
                             const TwoAxleSteeringCommand & currentCommand,
                             const double & dt);


}
#endif
