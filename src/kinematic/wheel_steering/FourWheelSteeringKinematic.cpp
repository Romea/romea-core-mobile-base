//romea
#include "romea_odo/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include "romea_odo/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"
#include "romea_odo/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"
#include "romea_odo/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include <romea_common/math/Algorithm.hpp>

//std
#include <cmath>
#include <iostream>

namespace romea {

//--------------------------------------------------------------------------
FourWheelSteeringKinematic::Parameters::Parameters():
  frontWheelBase(0),
  rearWheelBase(0),
  track(0),
  hubCarrierOffset(0),
  maximalWheelSpeed(std::numeric_limits<double>::max()),
  maximalWheelAcceleration(std::numeric_limits<double>::max()),
  maximalWheelAngle(M_PI_2),
  maximalWheelAngularSpeed(std::numeric_limits<double>::max()),
  wheelSpeedVariance(0),
  wheelAngleVariance(0)

{

}


//--------------------------------------------------------------------------
double FourWheelSteeringKinematic::comptuteBeta(const double & linearSpeedXBodyAxis,
                                                const double & linearSpeedYBodyAxis)
{
  return std::atan2(linearSpeedYBodyAxis,std::abs(linearSpeedXBodyAxis));
}


//--------------------------------------------------------------------------
double FourWheelSteeringKinematic::comptuteOrthogonalInstantaneousCurvature(const double & instantaneousCurvature,
                                                                            const double & beta)
{
  return instantaneousCurvature/std::cos(beta);
}

//--------------------------------------------------------------------------
double FourWheelSteeringKinematic::computeFrontSteeringAngle(const double & instantaneousCurvature,
                                                             const double & frontWheelBase,
                                                             const double & beta)
{
  return std::atan2(instantaneousCurvature*frontWheelBase+std::sin(beta),std::cos(beta));
}

//--------------------------------------------------------------------------
double FourWheelSteeringKinematic::computeRearSteeringAngle(const double & instantaneousCurvature,
                                                            const double & rearWheelBase,
                                                            const double & beta)
{
  return std::atan2(-instantaneousCurvature*rearWheelBase+std::sin(beta),std::cos(beta));
}

//--------------------------------------------------------------------------
TwoAxleSteeringCommand clamp(const FourWheelSteeringKinematic::Parameters & parameters,
                             const TwoAxleSteeringConstraints & userConstraints,
                             const TwoAxleSteeringCommand & command)
{

  return TwoAxleSteeringKinematic::clamp(parameters.frontWheelBase,
                                         parameters.rearWheelBase,
                                         parameters.track/2.,
                                         parameters.track/2.,
                                         parameters.hubCarrierOffset,
                                         parameters.hubCarrierOffset,
                                         parameters.maximalWheelSpeed,
                                         parameters.maximalWheelSpeed,
                                         parameters.maximalWheelAngle,
                                         parameters.maximalWheelAngle,
                                         userConstraints,
                                         command);
}

//--------------------------------------------------------------------------
TwoAxleSteeringCommand clamp(const FourWheelSteeringKinematic::Parameters & parameters,
                             const TwoAxleSteeringCommand & previousCommand,
                             const TwoAxleSteeringCommand & curentCommand,
                             const double & dt)
{

    double  maximalSteeringAngularSpeed =0;

    return TwoAxleSteeringKinematic::clamp(parameters.frontWheelBase,
                                           parameters.rearWheelBase,
                                           parameters.track/2.,
                                           parameters.track/2.,
                                           parameters.hubCarrierOffset,
                                           parameters.hubCarrierOffset,
                                           parameters.maximalWheelAcceleration,
                                           maximalSteeringAngularSpeed,
                                           previousCommand,
                                           curentCommand,
                                           dt);

}


////--------------------------------------------------------------------------
//double maximalPermissibleLinearSpeed(const FourWheelSteeringKinematic::Parameters & parameters,
//                                     const double & instantaneousCurvature)
//{

//}

////--------------------------------------------------------------------------
//double maximalPermissibleInstantaneousCurvature(const FourWheelSteeringKinematic::Parameters & parameters,
//                                                const double & linearSpeed)
//{

//}


////--------------------------------------------------------------------------
//double FourWheelSteeringKinematic::computeMaximalInstantaneousCurvature()const
//{
//  double wheelBase = getWheelBase("wheelbase");
//  const double frontTrack = getTrack("front_track");
//  const double rearTrack = getTrack("rear_track");
//  const double track = (frontTrack+rearTrack)/2.;

//  double maximalAbsoluteIntantaneousCurvature=OneAxleSteeringKinematic::
//      computeInstantaneousCurvature(maximalWheelAngle_,wheelBase/2.0);

//  maximalAbsoluteIntantaneousCurvature= maximalAbsoluteIntantaneousCurvature
//      /(1+maximalAbsoluteIntantaneousCurvature*track/2.0);

//  return maximalAbsoluteIntantaneousCurvature;

//}

////--------------------------------------------------------------------------
//double FourWheelSteeringKinematic::computeMaximalSteeringAngle()const
//{
//  double wheelBase = getWheelBase("wheelbase");
//  return OneAxleSteeringKinematic::computeSteeringAngle(computeMaximalInstantaneousCurvature(),wheelBase/2.0);
//}

////--------------------------------------------------------------------------
//KinematicCommand FourWheelSteeringKinematic::clamp(const KinematicCommand & command,
//                                                   const KinematicConstraints &userConstraints)const
//{


//  //clamp lateral speed
//  //double maximalAbsoluteBeta = maximalWheelAngle_;

//  double maximalAbsoluteLateralSpeed = maximalWheelSpeed_;
//  maximalAbsoluteLateralSpeed = std::min(maximalAbsoluteLateralSpeed,
//                                         userConstraints.getMaximalAbsoluteLinearSpeedAlongYBodyAxis());

//  double lateralSpeed = romea::clamp(command.getLinearSpeedAlongYBodyAxis(),
//                                         -maximalAbsoluteLateralSpeed,
//                                         maximalAbsoluteLateralSpeed);

//  //clamp curvature
//  double cosBeta = std::cos(std::atan(lateralSpeed/std::abs(command.getLinearSpeedAlongXBodyAxis())));

//  double maximalAbsoluteInstantaneousCurvature = 2*cosBeta/track_;
//  maximalAbsoluteInstantaneousCurvature=std::min(maximalAbsoluteInstantaneousCurvature,
//                                                 userConstraints.getMaximalAbsoluteInstantaneousCurvature());

//  double instantaneousCurvature =romea::clamp(command.getInstantaneousCurvature(),
//                                                  -maximalAbsoluteInstantaneousCurvature,
//                                                  maximalAbsoluteInstantaneousCurvature);


//  //clamp linear speed

//  double maximalAbsoluteLinearSpeed= maximalWheelSpeed_*std::cos(wheelAngle)/
//                                     (1 + std::abs(instantaneousCurvature)*track/(2*std::cos(beta)));

//  double absoluteMaximalAngularSpeed = userConstraints.getMaximalAbsoluteAngularSpeedAroundZBodyAxis();
//  if(absoluteMaximalAngularSpeed < std::abs(instantaneousCurvature)*maximalAbsoluteLinearSpeed)
//    maximalAbsoluteLinearSpeed = absoluteMaximalAngularSpeed/std::abs(instantaneousCurvature);

//  double minimalLinearSpeed = std::min(-maximalAbsoluteLinearSpeed,
//                                       userConstraints.getMinimalLinearSpeedAlongXBodyAxis());

//  double maximalLinearSpeed = std::min(maximalAbsoluteLinearSpeed,
//                                       userConstraints.getMaximalLinearSpeedAlongXBodyAxis());


//  double linearSpeed = romea::clamp(command.getLinearSpeedAlongXBodyAxis(),
//                                        minimalLinearSpeed,
//                                        maximalLinearSpeed);


//  //make command
//  return makeFourWheelSteeringCommand(command.getDuration(),linearSpeed,lateralSpeed,instantaneousCurvature);
//}


}//end romea
