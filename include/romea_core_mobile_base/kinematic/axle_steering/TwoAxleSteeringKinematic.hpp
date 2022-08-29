#ifndef romea_TwoAxleSteeringKinematic_hpp
#define romea_TwoAxleSteeringKinematic_hpp

//romea
#include "TwoAxleSteeringCommand.hpp"
#include "TwoAxleSteeringCommandLimits.hpp"

//std
#include <cmath>

namespace romea {

struct TwoAxleSteeringKinematic
{


  struct Parameters
  {
    Parameters();

//    enum Geometry{
//      ACKERMANN,
//      FOUR_WHEEL_STEERING
//    }

    double frontWheelBase;
    double rearWheelBase;
    double frontWheelTrack;
    double rearWheelTrack;
    double frontHubCarrierOffset;
    double rearHubCarrierOffset;
    double frontMaximalWheelLinearSpeed;
    double rearMaximalWheelLinearSpeed;
    double maximalWheelLinearAcceleration;
    double frontMaximalSteeringAngle;
    double rearMaximalSteeringAngle;
    double maximalSteeringAngularSpeed;
    double wheelLinearSpeedVariance;
    double steeringAngleVariance;
//    Geometry geometry;

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
                                      const TwoAxleSteeringCommandLimits & userLimits,
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
                             const TwoAxleSteeringCommandLimits & userLimits,
                             const TwoAxleSteeringCommand & command);

TwoAxleSteeringCommand clamp(const TwoAxleSteeringKinematic::Parameters & parameters,
                             const TwoAxleSteeringCommand & previousCommand,
                             const TwoAxleSteeringCommand & currentCommand,
                             const double & dt);


}
#endif
