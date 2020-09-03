//romea
#include "romea_odo/kinematic/skid_steering/SkidSteeringKinematic.hpp"
#include "romea_odo/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp"
#include "romea_odo/kinematic/axle_steering/OneAxleSteeringKinematic.hpp"
#include "romea_odo/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp"
#include "romea_odo/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"
#include <romea_common/math/Algorithm.hpp>

//std
#include <cmath>
#include <iostream>

namespace romea {


//--------------------------------------------------------------------------
TwoAxleSteeringKinematic::Parameters::Parameters():
  frontWheelBase(0),
  rearWheelBase(0),
  frontTrack(0),
  rearTrack(0),
  frontHubCarrierOffset(0),
  rearHubCarrierOffset(0),
  frontMaximalWheelSpeed(std::numeric_limits<double>::max()),
  rearMaximalWheelSpeed(std::numeric_limits<double>::max()),
  maximalWheelAcceleration(std::numeric_limits<double>::max()),
  frontMaximalSteeringAngle(M_PI_2),
  rearMaximalSteeringAngle(M_PI_2),
  maximalSteeringAngularSpeed(std::numeric_limits<double>::max()),
  wheelSpeedVariance(0),
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
//                                                           const KinematicConstraints &userConstraints)const
//{

//  assert(userConstraints.isValidInstantaneousCurvature(instantaneousCurvature));

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
//  double maximalAngularSpeed = userConstraints.getMaximalAbsoluteAngularSpeed();
//  if(maximalAngularSpeed< absoluteInstantaneousCurvature * maximalLinearSpeed)
//    maximalLinearSpeed = maximalAngularSpeed/absoluteInstantaneousCurvature;

//  return maximalLinearSpeed;
//}

////--------------------------------------------------------------------------
//double TwoAxleSteeringKinematic::computeMaximalInstantaneousCurvature(const double & linearSpeed,
//                                                                      const KinematicConstraints & userConstraints)const
//{

//  userConstraints.isValidSpeed(linearSpeed);

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
//                                                userConstraints.getMaximalAbsoluteInstantaneousCurvature());


//  double rearWheelBase = getWheelBase("rear_wheelBase");
//  double rearTrack = getTrack("rear_track");
//  double ar = rearWheelBase*rearWheelBase + rearTrack+rearTrack/4.;
//  double br = rearTrack;
//  double cr = 1 - (maximalWheelSpeed_*maximalWheelSpeed_)/(absoluteLinearSpeed*absoluteLinearSpeed);
//  double dr = br*br - 4*ar*cr;

//  double rearMaximalInstantaneousCurvature = (-br + std::sqrt(dr))/(2*ar);
//  rearMaximalInstantaneousCurvature = std::min(rearMaximalInstantaneousCurvature,
//                                               userConstraints.getMaximalAbsoluteInstantaneousCurvature());


//  double maximalInstantaneousCurvature = std::min(frontMaximalInstantaneousCurvature,
//                                                  rearMaximalInstantaneousCurvature);

//  //saturation according maximal angular speed
//  double maximalAngularSpeed = userConstraints.getMaximalAbsoluteAngularSpeed();
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
                                                       const TwoAxleSteeringConstraints & userConstraints,
                                                       const TwoAxleSteeringCommand & command)
{
  //clamp steering angle
  double maximalAbsoluteFrontSteeringAngle = std::min(frontMaximalSteeringAngle,
                                                      userConstraints.getMaximalAbsoluteFrontSteeringAngle());


  double frontSteeringAngle =romea::clamp(command.frontSteeringAngle,
                                          -maximalAbsoluteFrontSteeringAngle,
                                          maximalAbsoluteFrontSteeringAngle);

  double maximalAbsoluteRearSteeringAngle = std::min(rearMaximalSteeringAngle,
                                                     userConstraints.getMaximalAbsoluteRearSteeringAngle());

  double rearSteeringAngle =romea::clamp(command.rearSteeringAngle,
                                         -maximalAbsoluteRearSteeringAngle,
                                         maximalAbsoluteRearSteeringAngle);


  //clamp linear speed
  double maximalAbsoluteLinearSpeed=std::numeric_limits<double>::max();


  double frontTanSteeringAngle = std::tan(frontSteeringAngle);
  double rearTanSteeringAngle = std::tan(rearSteeringAngle);
  double instantaneousCurvature = (frontTanSteeringAngle - rearTanSteeringAngle)/(frontWheelBase+rearWheelBase);


  double frontLeftRatio = OneAxleSteeringKinematic::computeWheelSpeedRatio(-frontTanSteeringAngle,-instantaneousCurvature,frontHubCarrierOffset,frontHalfTrack);
  double frontRigthRatio = OneAxleSteeringKinematic::computeWheelSpeedRatio(frontTanSteeringAngle,instantaneousCurvature,frontHubCarrierOffset,frontHalfTrack);



  maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,frontMaximalWheelSpeed/frontLeftRatio);
  maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,frontMaximalWheelSpeed/frontRigthRatio);

  double rearLeftRatio = OneAxleSteeringKinematic::computeWheelSpeedRatio(-rearTanSteeringAngle,-instantaneousCurvature,rearHubCarrierOffset,rearHalfTrack);
  double rearRightRatio = OneAxleSteeringKinematic::computeWheelSpeedRatio(rearTanSteeringAngle,instantaneousCurvature,rearHubCarrierOffset,rearHalfTrack);



  maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,rearMaximalWheelSpeed/rearLeftRatio);
  maximalAbsoluteLinearSpeed = std::min(maximalAbsoluteLinearSpeed,rearMaximalWheelSpeed/rearRightRatio);

  double minimalLinearSpeed = std::max(-maximalAbsoluteLinearSpeed,
                                       userConstraints.getMinimalLinearSpeed());

  double maximalLinearSpeed = std::min(maximalAbsoluteLinearSpeed,
                                       userConstraints.getMaximalLinearSpeed());

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
TwoAxleSteeringCommand clamp(const TwoAxleSteeringKinematic::Parameters & parameters,
                             const TwoAxleSteeringConstraints & userConstraints,
                             const TwoAxleSteeringCommand & command)
{

  return TwoAxleSteeringKinematic::clamp(parameters.frontWheelBase,
                                         parameters.rearWheelBase,
                                         parameters.frontTrack/2.,
                                         parameters.rearTrack/2.,
                                         parameters.frontHubCarrierOffset,
                                         parameters.rearHubCarrierOffset,
                                         parameters.frontMaximalWheelSpeed,
                                         parameters.rearMaximalWheelSpeed,
                                         parameters.frontMaximalSteeringAngle,
                                         parameters.rearMaximalSteeringAngle,
                                         userConstraints,
                                         command);
}

}//end romea
