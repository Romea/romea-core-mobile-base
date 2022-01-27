//romea
#include "romea_core_odo/kinematic/skid_steering/InverseSkidSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>
#include <iostream>
namespace {

//const double LINEAR_SPEED_EPSILON = 0.00001;

////--------------------------------------------------------------------------
//inline Eigen::Matrix4d computeCovarianceMatrix(const double & leftWheelSpeed,
//                                               const double & rightWheelSpeed,
//                                               const double & wheelSpeedVariance,
//                                               const double & track)
//{
//  Eigen::Matrix4d covarianceMatrix = Eigen::Matrix4d::Zero();

//  double gamma = track*std::pow((leftWheelSpeed + rightWheelSpeed)/2.,2);
//  double alpha = (leftWheelSpeed)/gamma;
//  double beta = -(rightWheelSpeed)/gamma;

//  covarianceMatrix(0,0)=0.5;
//  covarianceMatrix(0,3)=0.5*(alpha + beta);
//  covarianceMatrix(3,0)=covarianceMatrix(0,3);
//  covarianceMatrix(2,2)=2/(track*track);
//  covarianceMatrix(2,3)=(alpha - beta)/track;
//  covarianceMatrix(3,2)=covarianceMatrix(2,3);
//  covarianceMatrix(3,3)= alpha*alpha + beta*beta;
//  covarianceMatrix*=wheelSpeedVariance;

//  return covarianceMatrix;
//}

////--------------------------------------------------------------------------
//inline Eigen::Matrix4d computeCovarianceMatrix(const double & wheelSpeedVariance,
//                                               const double & track)
//{
//  Eigen::Matrix4d covarianceMatrix = Eigen::Matrix4d::Zero();

//  covarianceMatrix(0,0)=0.5;
//  covarianceMatrix(2,2)=2/(track*track);
//  covarianceMatrix(3,3)=std::numeric_limits<double>::max();
//  covarianceMatrix*=wheelSpeedVariance;
//  return covarianceMatrix;
//}

//--------------------------------------------------------------------------
void inverseKinematicImpl(const double & wheelTrack,
                          const double & leftWheelSpeed,
                          const double & rightWheelSpeed,
                          const double & wheelSpeedVariance,
                          romea::SkidSteeringMeasure & skidSteeringMeasure)
{
  skidSteeringMeasure.longitudinalSpeed = romea::SkidSteeringKinematic::computeLinearSpeed(leftWheelSpeed,rightWheelSpeed);
  skidSteeringMeasure.angularSpeed = romea::SkidSteeringKinematic::computeAngularSpeed(leftWheelSpeed,rightWheelSpeed,wheelTrack);
  skidSteeringMeasure.covariance << 0.5, 1/wheelTrack, 1/wheelTrack, 1/(wheelTrack*wheelTrack);
  skidSteeringMeasure.covariance*= wheelSpeedVariance;
}


}

namespace romea {


////-----------------------------------------------------------------------------
//KinematicMeasure inverseKinematic2WD(const SkidSteeringKinematic & kinematic,
//                                     const OdometryFrame2WD & odometryFrame,
//                                     const double & wheelSpeedVariance)
//{
//  const double & track= kinematic.getTrack("front_track");
//  const double & leftWheelSpeed = odometryFrame.leftWheelSpeed;
//  const double & rightWheelSpeed = odometryFrame.rightWheelSpeed;

//  double linearSpeed = SkidSteeringKinematic::computeLinearSpeed(leftWheelSpeed,rightWheelSpeed);
//  double angularSpeed = SkidSteeringKinematic::computeAngularSpeed(leftWheelSpeed,rightWheelSpeed,track);
//  double instantaneousCurvature = SkidSteeringKinematic::computeInstantaneousCurvature(leftWheelSpeed,rightWheelSpeed,track);

//  //Create kinematic frame
//  KinematicMeasure kinematicMeasure;
//  kinematicMeasure.speed=linearSpeed;
//  kinematicMeasure.beta=0.;
//  kinematicMeasure.angularSpeed=angularSpeed;

//  if(std::abs(linearSpeed)> LINEAR_SPEED_EPSILON){

//    kinematicMeasure.instantaneousCurvature=instantaneousCurvature;
//    kinematicMeasure.covariance =computeCovarianceMatrix(leftWheelSpeed,rightWheelSpeed,wheelSpeedVariance,track);

//  }else{

//    kinematicMeasure.instantaneousCurvature=sign(angularSpeed)*std::numeric_limits<double>::max();
//    kinematicMeasure.covariance = computeCovarianceMatrix(wheelSpeedVariance,track);

//  }
//  return kinematicMeasure;
//}


//-----------------------------------------------------------------------------
void inverseKinematic(const SkidSteeringKinematic::Parameters &parameters,
                      const OdometryFrame2WD & odometryFrame,
                      SkidSteeringMeasure & skidSteeringMeasure)
{
  inverseKinematicImpl(parameters.wheelTrack,
                       odometryFrame.leftWheelSpeed,
                       odometryFrame.rightWheelSpeed,
                       parameters.wheelSpeedVariance,
                       skidSteeringMeasure);

}


////-----------------------------------------------------------------------------
//KinematicMeasure inverseKinematic4WD(const SkidSteeringKinematic & kinematic,
//                                     const OdometryFrame4WD & odometryFrame,
//                                     const double & wheelSpeedVariance)
//{
//  const double track= kinematic.getTrack("front_track");

//  double leftWheelSpeed = SkidSteeringKinematic::minWheelSpeed(odometryFrame.frontLeftWheelSpeed,
//                                                               odometryFrame.rearLeftWheelSpeed);

//  double rightWheelSpeed =SkidSteeringKinematic:: minWheelSpeed(odometryFrame.frontRightWheelSpeed,
//                                                                odometryFrame.rearRightWheelSpeed);


//  double linearSpeed = SkidSteeringKinematic::computeLinearSpeed(leftWheelSpeed,rightWheelSpeed);
//  double angularSpeed = SkidSteeringKinematic::computeAngularSpeed(leftWheelSpeed,rightWheelSpeed,track);
//  double instantaneousCurvature = SkidSteeringKinematic::computeInstantaneousCurvature(leftWheelSpeed,rightWheelSpeed,track);

//  KinematicMeasure kinematicMeasure;
//  kinematicMeasure.speed=linearSpeed;
//  kinematicMeasure.beta=0.;
//  kinematicMeasure.angularSpeed=angularSpeed;

//  if(std::abs(linearSpeed)> LINEAR_SPEED_EPSILON){

//    kinematicMeasure.instantaneousCurvature=instantaneousCurvature;
//    kinematicMeasure.covariance =computeCovarianceMatrix(leftWheelSpeed,rightWheelSpeed,wheelSpeedVariance,track);

//  }else{

//    kinematicMeasure.instantaneousCurvature=sign(angularSpeed)*std::numeric_limits<double>::max();
//    kinematicMeasure.covariance = computeCovarianceMatrix(wheelSpeedVariance,track);

//  }

//  return kinematicMeasure;
//}

//-----------------------------------------------------------------------------
void inverseKinematic(const SkidSteeringKinematic::Parameters &parameters,
                      const OdometryFrame4WD & odometryFrame,
                      SkidSteeringMeasure & skidSteeringMeasure)
{
  double leftWheelSpeed = SkidSteeringKinematic::minWheelSpeed(odometryFrame.frontLeftWheelSpeed,
                                                               odometryFrame.rearLeftWheelSpeed);

  double rightWheelSpeed =SkidSteeringKinematic:: minWheelSpeed(odometryFrame.frontRightWheelSpeed,
                                                                odometryFrame.rearRightWheelSpeed);


  inverseKinematicImpl(parameters.wheelTrack,
                       leftWheelSpeed,
                       rightWheelSpeed,
                       parameters.wheelSpeedVariance,
                       skidSteeringMeasure);
}

}//end romea
