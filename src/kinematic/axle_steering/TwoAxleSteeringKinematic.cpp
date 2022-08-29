//romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

//std
#include <cmath>
#include <iostream>

namespace romea {


//--------------------------------------------------------------------------
TwoAxleSteeringKinematic::Parameters::Parameters():
    frontWheelBase(0),
    rearWheelBase(0),
    frontWheelTrack(0),
    rearWheelTrack(0),
    frontHubCarrierOffset(0),
    rearHubCarrierOffset(0),
    frontMaximalWheelLinearSpeed(std::numeric_limits<double>::max()),
    rearMaximalWheelLinearSpeed(std::numeric_limits<double>::max()),
    maximalWheelLinearAcceleration(std::numeric_limits<double>::max()),
    frontMaximalSteeringAngle(M_PI_2),
    rearMaximalSteeringAngle(M_PI_2),
    maximalSteeringAngularSpeed(std::numeric_limits<double>::max()),
    wheelLinearSpeedVariance(0),
    steeringAngleVariance(0)
{

}


//--------------------------------------------------------------------------
double TwoAxleSteeringKinematic::computeBeta(const double & tanFrontSteeringAngle,
                                             const double & tanRearSteeringAngle,
                                             const double & frontWheelBase,
                                             const double & rearWheelBase)
{
    return std::atan((rearWheelBase*tanFrontSteeringAngle + frontWheelBase*tanRearSteeringAngle)/
                     (frontWheelBase+rearWheelBase));
}


//--------------------------------------------------------------------------
double TwoAxleSteeringKinematic::computeOrthogonalInstantaneousCurvature(const double & tanFrontSteeringAngle,
                                                                         const double & tanRearSteeringAngle,
                                                                         const double & frontWheelBase,
                                                                         const double & rearWheelBase)
{
    return (tanFrontSteeringAngle-tanRearSteeringAngle)/(frontWheelBase+rearWheelBase);
}

//--------------------------------------------------------------------------
double TwoAxleSteeringKinematic::computeInstantaneousCurvature(const double & tanFrontSteeringAngle,
                                                               const double & tanRearSteeringAngle,
                                                               const double & frontWheelBase,
                                                               const double & rearWheelBase)
{

    double K=computeOrthogonalInstantaneousCurvature(tanFrontSteeringAngle,
                                                     tanRearSteeringAngle,
                                                     frontWheelBase,
                                                     rearWheelBase);

    double beta = computeBeta(tanFrontSteeringAngle,
                              tanRearSteeringAngle,
                              frontWheelBase,
                              rearWheelBase);

    return K*std::sqrt(1+std::pow(std::tan(beta),2));
}




//-----------------------------------------------------------------------------
TwoAxleSteeringCommand TwoAxleSteeringKinematic::clamp(const double  & frontWheelBase,
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
                                                       const TwoAxleSteeringCommand & command)
{
    //clamp steering angle
    double maximalAbsoluteFrontSteeringAngle = std::min(frontMaximalSteeringAngle,
                                                        userLimits.frontSteeringAngle.upper());


    double frontSteeringAngle =romea::clamp(command.frontSteeringAngle,
                                            -maximalAbsoluteFrontSteeringAngle,
                                            maximalAbsoluteFrontSteeringAngle);

    double maximalAbsoluteRearSteeringAngle = std::min(rearMaximalSteeringAngle,
                                                       userLimits.rearSteeringAngle.upper());

    double rearSteeringAngle =romea::clamp(command.rearSteeringAngle,
                                           -maximalAbsoluteRearSteeringAngle,
                                           maximalAbsoluteRearSteeringAngle);


    //clamp linear speed
    double maximalAbsoluteLinearSpeed=std::numeric_limits<double>::max();

    double frontTanSteeringAngle = std::tan(frontSteeringAngle);
    double rearTanSteeringAngle = std::tan(rearSteeringAngle);
    double instantaneousCurvature = (frontTanSteeringAngle - rearTanSteeringAngle)/(frontWheelBase+rearWheelBase);

    double frontLeftRatio = OneAxleSteeringKinematic::computeWheelLinearSpeedRatio(-frontTanSteeringAngle,-instantaneousCurvature,frontHubCarrierOffset,frontHalfTrack);
    double frontRigthRatio = OneAxleSteeringKinematic::computeWheelLinearSpeedRatio(frontTanSteeringAngle,instantaneousCurvature,frontHubCarrierOffset,frontHalfTrack);

    maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,frontMaximalWheelSpeed/frontLeftRatio);
    maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,frontMaximalWheelSpeed/frontRigthRatio);

    double rearLeftRatio = OneAxleSteeringKinematic::computeWheelLinearSpeedRatio(-rearTanSteeringAngle,-instantaneousCurvature,rearHubCarrierOffset,rearHalfTrack);
    double rearRightRatio = OneAxleSteeringKinematic::computeWheelLinearSpeedRatio(rearTanSteeringAngle,instantaneousCurvature,rearHubCarrierOffset,rearHalfTrack);

    maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,rearMaximalWheelSpeed/rearLeftRatio);
    maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,rearMaximalWheelSpeed/rearRightRatio);

    double minimalLinearSpeed = std::max(-maximalAbsoluteLinearSpeed,
                                         userLimits.longitudinalSpeed.lower());

    double maximalLinearSpeed = std::min(maximalAbsoluteLinearSpeed,
                                         userLimits.longitudinalSpeed.upper());

    double linearSpeed = romea::clamp(command.longitudinalSpeed,
                                      minimalLinearSpeed,
                                      maximalLinearSpeed);


    TwoAxleSteeringCommand clampedCommand;
    clampedCommand.longitudinalSpeed = linearSpeed;
    clampedCommand.frontSteeringAngle = frontSteeringAngle;
    clampedCommand.rearSteeringAngle = rearSteeringAngle;
    return clampedCommand;
}


//-----------------------------------------------------------------------------
TwoAxleSteeringCommand TwoAxleSteeringKinematic::clamp(const double  & frontWheelBase,
                                                       const double  & rearWheelBase,
                                                       const double  & frontHalfTrack,
                                                       const double  & rearHalfTrack,
                                                       const double  & frontHubCarrierOffset,
                                                       const double  & rearHubCarrierOffset,
                                                       const double  & maximalWheelAcceleration,
                                                       const double  & maximalSteeringAngularSpeed,
                                                       const TwoAxleSteeringCommand & previousCommand,
                                                       const TwoAxleSteeringCommand & currentCommand,
                                                       const double & dt)
{

    //clamp steering angular speed
    double frontSteeringAngularSpeed = currentCommand.frontSteeringAngle-previousCommand.frontSteeringAngle;

    frontSteeringAngularSpeed =romea::clamp(frontSteeringAngularSpeed,
                                            -maximalSteeringAngularSpeed,
                                            maximalSteeringAngularSpeed);

    double rearSteeringAngularSpeed = currentCommand.rearSteeringAngle-previousCommand.rearSteeringAngle;

    rearSteeringAngularSpeed =romea::clamp(rearSteeringAngularSpeed,
                                           -maximalSteeringAngularSpeed,
                                           maximalSteeringAngularSpeed);


    //clamp linear speed
    double maximalLinearAcceleration=std::numeric_limits<double>::max();

    double frontTanSteeringAngle = std::tan(previousCommand.frontSteeringAngle);
    double rearTanSteeringAngle = std::tan(previousCommand.rearSteeringAngle);
    double instantaneousCurvature = (frontTanSteeringAngle - rearTanSteeringAngle)/(frontWheelBase+rearWheelBase);

    double frontLeftRatio = OneAxleSteeringKinematic::computeWheelLinearSpeedRatio(-frontTanSteeringAngle,-instantaneousCurvature,frontHubCarrierOffset,frontHalfTrack);
    double frontRigthRatio = OneAxleSteeringKinematic::computeWheelLinearSpeedRatio(frontTanSteeringAngle,instantaneousCurvature,frontHubCarrierOffset,frontHalfTrack);

    maximalLinearAcceleration = std::min(maximalLinearAcceleration,maximalWheelAcceleration/frontLeftRatio);
    maximalLinearAcceleration = std::min(maximalLinearAcceleration,maximalWheelAcceleration/frontRigthRatio);

    double rearLeftRatio = OneAxleSteeringKinematic::computeWheelLinearSpeedRatio(-rearTanSteeringAngle,-instantaneousCurvature,rearHubCarrierOffset,rearHalfTrack);
    double rearRightRatio = OneAxleSteeringKinematic::computeWheelLinearSpeedRatio(rearTanSteeringAngle,instantaneousCurvature,rearHubCarrierOffset,rearHalfTrack);

    maximalLinearAcceleration = std::min(maximalLinearAcceleration,maximalWheelAcceleration/rearLeftRatio);
    maximalLinearAcceleration = std::min(maximalLinearAcceleration,maximalWheelAcceleration/rearRightRatio);


    //return command
    TwoAxleSteeringCommand clampedCommand;
    clampedCommand.longitudinalSpeed = previousCommand.longitudinalSpeed + maximalLinearAcceleration*dt;
    clampedCommand.frontSteeringAngle = previousCommand.frontSteeringAngle + frontSteeringAngularSpeed*dt;
    clampedCommand.rearSteeringAngle = previousCommand.rearSteeringAngle +rearSteeringAngularSpeed*dt;
    return clampedCommand;

}

//-----------------------------------------------------------------------------
TwoAxleSteeringCommand clamp(const TwoAxleSteeringKinematic::Parameters & parameters,
                             const TwoAxleSteeringCommandLimits & userLimits,
                             const TwoAxleSteeringCommand & command)
{

    return TwoAxleSteeringKinematic::clamp(parameters.frontWheelBase,
                                           parameters.rearWheelBase,
                                           parameters.frontWheelTrack/2.,
                                           parameters.rearWheelTrack/2.,
                                           parameters.frontHubCarrierOffset,
                                           parameters.rearHubCarrierOffset,
                                           parameters.frontMaximalWheelLinearSpeed,
                                           parameters.rearMaximalWheelLinearSpeed,
                                           parameters.frontMaximalSteeringAngle,
                                           parameters.rearMaximalSteeringAngle,
                                           userLimits,
                                           command);
}

//-----------------------------------------------------------------------------
TwoAxleSteeringCommand clamp(const TwoAxleSteeringKinematic::Parameters & parameters,
                             const TwoAxleSteeringCommand & previousCommand,
                             const TwoAxleSteeringCommand & currentCommand,
                             const double & dt)
{
    return TwoAxleSteeringKinematic::clamp(parameters.frontWheelBase,
                                           parameters.rearWheelBase,
                                           parameters.frontWheelTrack/2.,
                                           parameters.rearWheelTrack/2.,
                                           parameters.frontHubCarrierOffset,
                                           parameters.rearHubCarrierOffset,
                                           parameters.maximalWheelLinearAcceleration,
                                           parameters.maximalSteeringAngularSpeed,
                                           previousCommand,
                                           currentCommand,
                                           dt);

}

////--------------------------------------------------------------------------
//double TwoAxleSteeringKinematic::computeFrontLeftWheelSpeed(const double & linearSpeedAlongXAxisBody,
//                                                            const double & orthognonalInstantaneousCurvature,
//                                                            const double & tanFrontSteeringAngle)
//{
//  return OneAxleSteeringKinematic::computeLeftWheelSpeed(linearSpeedAlongXAxisBody,
//                                                         orthognonalInstantaneousCurvature,
//                                                         tanFrontSteeringAngle);

//}



////--------------------------------------------------------------------------
//double TwoAxleSteeringKinematic::computeFrontRightWheelSpeed(const double & linearSpeedAlongXAxisBody,
//                                                             const double & orthognonalInstantaneousCurvature,
//                                                             const double & tanFrontSteeringAngle)
//{
//  return OneAxleSteeringKinematic::computeRightWheelSpeed(linearSpeedAlongXAxisBody,
//                                                          orthognonalInstantaneousCurvature,
//                                                          tanFrontSteeringAngle);

//}

////--------------------------------------------------------------------------
//double TwoAxleSteeringKinematic::computeRearLeftWheelSpeed(const double & linearSpeedAlongXAxisBody,
//                                                           const double & orthognonalInstantaneousCurvature,
//                                                           const double & tanRearSteeringAngle)
//{
//  return OneAxleSteeringKinematic::computeLeftWheelSpeed(linearSpeedAlongXAxisBody,
//                                                         orthognonalInstantaneousCurvature,
//                                                         tanRearSteeringAngle);
//}


////--------------------------------------------------------------------------
//double TwoAxleSteeringKinematic::computeRearRightWheelSpeed(const double & linearSpeedAlongXAxisBody,
//                                                            const double & orthognonalInstantaneousCurvature,
//                                                            const double & tanRearSteeringAngle)
//{
//  return OneAxleSteeringKinematic::computeRightWheelSpeed(linearSpeedAlongXAxisBody,
//                                                          orthognonalInstantaneousCurvature,
//                                                          tanRearSteeringAngle);

//}


////--------------------------------------------------------------------------
//double TwoAxleSteeringKinematic::computeMaximalLinearSpeed(const double & instantaneousCurvature,
//                                                           const KinematicConstraints &userLimits)const
//{

//  assert(userLimits.isValidInstantaneousCurvature(instantaneousCurvature));

//  double absoluteInstantaneousCurvature = std::abs(instantaneousCurvature);
//  if( absoluteInstantaneousCurvature < std::numeric_limits<float>::epsilon() )
//    return maximalWheelSpeed_;

//  //For linearSpeed > 0 && instantaneousCurvature > 0
//  //Right wheel speed is higher than left wheel speed
//  //So we consider only right wheel

//  //We don't use this code
//  //double steeringAngle = computeSteeringAngle(absoluteInstantaneousCurvature);
//  //double rightWheelAngle = computeRightWheelAngle(absoluteInstantaneousCurvature);
//  //double maximalLinearSpeed = maximalWheelSpeed_ /std::tan(steeringAngle)*std::sin(rightWheelAngle);
//  //But this one in order to the exact inverse function of computeMaximalInstantaneousCurvatureAccordingLinearSpeed

//  double frontWheelBase = getWheelBase("front_wheelbase");
//  double frontTrack = getTrack("front_track");
//  double af = frontWheelBase*frontWheelBase + frontTrack+frontTrack/4.;
//  double bf = frontTrack;
//  double cf = 1;

//  double frontMaximalLinearSpeed = maximalWheelSpeed_
//      /sqrt(af*absoluteInstantaneousCurvature*absoluteInstantaneousCurvature+
//            bf*absoluteInstantaneousCurvature +cf);

//  double rearWheelBase = getWheelBase("rear_wheelbase");
//  double rearTrack = getTrack("rear_track");
//  double ar = rearWheelBase*rearWheelBase + rearTrack+rearTrack/4.;
//  double br = rearTrack;
//  double cr = 1;

//  double rearMaximalLinearSpeed = maximalWheelSpeed_
//      /sqrt(ar*absoluteInstantaneousCurvature*absoluteInstantaneousCurvature+
//            br*absoluteInstantaneousCurvature +cr);

//  double maximalLinearSpeed =std::min(frontMaximalLinearSpeed,rearMaximalLinearSpeed);

//  //saturation according maximal angular speed
//  double maximalAngularSpeed = userLimits.getMaximalAbsoluteAngularSpeed();
//  if(maximalAngularSpeed< absoluteInstantaneousCurvature * maximalLinearSpeed)
//    maximalLinearSpeed = maximalAngularSpeed/absoluteInstantaneousCurvature;

//  return maximalLinearSpeed;
//}

////--------------------------------------------------------------------------
//double TwoAxleSteeringKinematic::computeMaximalInstantaneousCurvature(const double & linearSpeed,
//                                                                      const KinematicConstraints & userLimits)const
//{

//  userLimits.isValidSpeed(linearSpeed);

//  double absoluteLinearSpeed = std::abs(linearSpeed);
//  if(absoluteLinearSpeed < std::numeric_limits<float>::epsilon())
//    return maximalWheelSpeed_;

//  //For linearSpeed > 0 && instantaneousCurvature > 0
//  //Right wheel speed is higher than left wheel speed
//  //So we consider only right wheel

//  //double steeringAngle = computeSteeringAngle(absoluteInstantaneousCurvature);
//  //double rightWheelAngle = computeRightWheelAngle(absoluteInstantaneousCurvature);
//  //We want to solve  vRightMax  - linearSpeed * tan(steeringAngle) / sin(rightWheelAngle)  = 0
//  //For this it means the following second order function

//  double frontWheelBase = getWheelBase("front_wheelbase");
//  double frontTrack = getTrack("front_track");
//  double af = frontWheelBase*frontWheelBase + frontTrack+frontTrack/4.;
//  double bf = frontTrack;
//  double cf = 1 - (maximalWheelSpeed_*maximalWheelSpeed_)/(absoluteLinearSpeed*absoluteLinearSpeed);
//  double df = bf*bf - 4*af*cf;

//  double frontMaximalInstantaneousCurvature = (-bf + std::sqrt(df))/(2*af);
//  frontMaximalInstantaneousCurvature = std::min(frontMaximalInstantaneousCurvature,
//                                                userLimits.getMaximalAbsoluteInstantaneousCurvature());


//  double rearWheelBase = getWheelBase("rear_wheelBase");
//  double rearTrack = getTrack("rear_track");
//  double ar = rearWheelBase*rearWheelBase + rearTrack+rearTrack/4.;
//  double br = rearTrack;
//  double cr = 1 - (maximalWheelSpeed_*maximalWheelSpeed_)/(absoluteLinearSpeed*absoluteLinearSpeed);
//  double dr = br*br - 4*ar*cr;

//  double rearMaximalInstantaneousCurvature = (-br + std::sqrt(dr))/(2*ar);
//  rearMaximalInstantaneousCurvature = std::min(rearMaximalInstantaneousCurvature,
//                                               userLimits.getMaximalAbsoluteInstantaneousCurvature());


//  double maximalInstantaneousCurvature = std::min(frontMaximalInstantaneousCurvature,
//                                                  rearMaximalInstantaneousCurvature);

//  //saturation according maximal angular speed
//  double maximalAngularSpeed = userLimits.getMaximalAbsoluteAngularSpeed();
//  if(maximalAngularSpeed< maximalInstantaneousCurvature * absoluteLinearSpeed)
//    maximalInstantaneousCurvature = maximalAngularSpeed/absoluteLinearSpeed;

//  return maximalInstantaneousCurvature;


//}

////-----------------------------------------------------------------------------
//double maximalPermissibleLinearSpeed(const TwoAxleSteeringKinematic::Parameters & parameters,
//                                     const double & instantaneousCurvature)
//{
////  double frontMaximalLinearSpeed= OneAxleSteeringKinematic::
////      maximalPermissibleLinearSpeed(parameters.frontWheelBase,
////                                    parameters.frontTrack,
////                                    parameters.frontMaximalWheelSpeed,
////                                    parameters.frontMaximalSteeringAngle,
////                                    parameters.frontHubCarrierOffset,
////                                    instantaneousCurvature);

////  double rearMaximalLinearSpeed= OneAxleSteeringKinematic::
////      maximalPermissibleLinearSpeed(parameters.rearWheelBase,
////                                    parameters.rearTrack,
////                                    parameters.rearMaximalWheelSpeed,
////                                    parameters.rearMaximalSteeringAngle,
////                                    parameters.rearHubCarrierOffset,
////                                    instantaneousCurvature);

////  return std::min(frontMaximalLinearSpeed,rearMaximalLinearSpeed);
//}

////-----------------------------------------------------------------------------
//double maximalPermissibleInstantaneousCurvature(const TwoAxleSteeringKinematic::Parameters & parameters,
//                                                const double & linearSpeed)
//{
////  double frontMaximalInstantaneousCurvature= OneAxleSteeringKinematic::
////      maximalPermissibleInstantaneousCurvature(parameters.frontWheelBase,
////                                               parameters.frontTrack,
////                                               parameters.frontMaximalWheelSpeed,
////                                               parameters.frontMaximalSteeringAngle,
////                                               parameters.frontHubCarrierOffset,
////                                               linearSpeed);

////  double rearMaximalInstantaneousCurvature= OneAxleSteeringKinematic::
////      maximalPermissibleInstantaneousCurvature(parameters.rearWheelBase,
////                                               parameters.rearTrack,
////                                               parameters.rearMaximalWheelSpeed,
////                                               parameters.rearMaximalSteeringAngle,
////                                               parameters.rearHubCarrierOffset,
////                                               linearSpeed);

////  return std::min(frontMaximalInstantaneousCurvature,rearMaximalInstantaneousCurvature);
//}

}//end romea
