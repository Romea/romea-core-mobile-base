#ifndef romea_SkidSteeringKinematic_hpp
#define romea_SkidSteeringKinematic_hpp

//romea
#include "SkidSteeringCommand.hpp"
#include "SkidSteeringConstraints.hpp"

namespace romea
{

struct SkidSteeringKinematic
{

  struct Parameters
  {
    Parameters();
    double track;
    double maximalWheelSpeed;
    double maximalWheelAcceleration;
    double wheelSpeedVariance;
  };

  
  static double computeLinearSpeed(const double &leftWheelSpeed,
                                   const double &rightWheelSpeed);

  static double computeAngularSpeed(const double &leftWheelSpeed,
                                    const double &rightWheelSpeed,
                                    const double & track);

  static double computeInstantaneousCurvature(const double &leftWheelSpeed,
                                              const double & rightWheelSpeed,
                                              const double & track);

  static double computeLeftWheelSpeed(const double & linearSpeed ,
                                      const double & angularSpeed,
                                      const double & track);

  static double computeRightWheelSpeed(const double & linearSpeed ,
                                       const double & angularSpeed,
                                       const double & track);

  static double minWheelSpeed(const double &frontWheelSpeed, const double & rearWheelSpeed);


};

SkidSteeringCommand clamp(const SkidSteeringKinematic::Parameters & parameters,
                          const SkidSteeringConstraints & userConstraints,
                          const SkidSteeringCommand & command);

SkidSteeringCommand clamp(const SkidSteeringKinematic::Parameters & parameters,
                          const SkidSteeringCommand & previousCommand,
                          const SkidSteeringCommand & currentCommand,
                          const double & dt);

}//end romea

#endif
