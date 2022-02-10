#ifndef romea_MecanumWheelSteeringKinematic_hpp
#define romea_MecanumWheelSteeringKinematic_hpp

//romea
#include "OmniSteeringCommand.hpp"
#include "OmniSteeringCommandLimits.hpp"

namespace romea
{

struct MecanumWheelSteeringKinematic
{

  struct Parameters
  {
    Parameters();
    double wheelTrack;
    double wheelbase;
    double maximalWheelSpeed;
    double maximalWheelAcceleration;
    double wheelSpeedVariance;
  };



  static double computeFrontLeftWheelSpeed(const double & longitidutinalSpeed,
                                           const double & lateralSpeed,
                                           const double & angularSpeed,
                                           const double & halfWheelbase,
                                           const double & halfTrack);

  static double computeFrontRightWheelSpeed(const double & longitidutinalSpeed,
                                            const double & lateralSpeed,
                                            const double & angularSpeed,
                                            const double & halfWheelbase,
                                            const double & halfTrack);

  static double computeRearLeftWheelSpeed(const double & longitidutinalSpeed,
                                          const double & lateralSpeed,
                                          const double & angularSpeed,
                                          const double & halfWheelbase,
                                          const double & halfTrack);

  static double computeRearRightWheelSpeed(const double & longitidutinalSpeed,
                                           const double & lateralSpeed,
                                           const double & angularSpeed,
                                           const double & halfWheelbase,
                                           const double & halfTrack);


  static double computeLongitudinalSpeed(const double & frontLeftWheelSpeed,
                                         const double & frontRightWheelSpeed,
                                         const double & rearLeftWheelSpeed,
                                         const double & rearRightWheelSpeed);

  static double computeLateralSpeed(const double & frontLeftWheelSpeed,
                                    const double & frontRightWheelSpeed,
                                    const double & rearLeftWheelSpeed,
                                    const double & rearRightWheelSpeed);


  static double computeAngularSpeed(const double & frontLeftWheelSpeed,
                                    const double & frontRightWheelSpeed,
                                    const double & rearLeftWheelSpeed,
                                    const double & rearRightWheelSpeed,
                                    const double & halfWheelbase,
                                    const double & halfTrack);

};

OmniSteeringCommand clamp(const MecanumWheelSteeringKinematic::Parameters & parameters,
                          const OmniSteeringCommandLimits & userLimits,
                          const OmniSteeringCommand & command);

OmniSteeringCommand clamp(const MecanumWheelSteeringKinematic::Parameters & parameters,
                          const OmniSteeringCommand & previousCommand,
                          const OmniSteeringCommand & currentCommand,
                          const double & dt);


}//end romea

#endif
