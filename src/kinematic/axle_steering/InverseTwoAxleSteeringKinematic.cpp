// romea
#include "romea_core_mobile_base/kinematic/axle_steering/InverseTwoAxleSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

namespace romea {

//-----------------------------------------------------------------------------
void inverseKinematic(const TwoAxleSteeringKinematic::Parameters & parameters,
                      const OdometryFrame2AS4WD & odometryFrame,
                      TwoAxleSteeringMeasure & twoAxleSteeringMeasure)
{
  const double &wheelSpeedVariance = parameters.wheelLinearSpeedVariance;
  const double &steeringAngleVariance = parameters.steeringAngleVariance;

  const double frontWheelbase = parameters.frontWheelBase;
  const double frontHalfWheelTrack = parameters.frontWheelTrack/2;
  const double frontHubCarrierOffset = parameters.frontHubCarrierOffset;
  const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelLinearSpeed;
  const double & frontRightWheelSpeed = odometryFrame.frontRightWheelLinearSpeed;
  const double & frontSteeringAngle = odometryFrame.frontAxleSteeringAngle;

  const double rearWheelbase = parameters.rearWheelBase;
  const double rearHalfTrack = parameters.rearWheelTrack/2;
  const double rearHubCarrierOffset = parameters.rearHubCarrierOffset;
  const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelLinearSpeed;
  const double & rearRightWheelSpeed = odometryFrame.rearRightWheelLinearSpeed;
  const double & rearSteeringAngle = odometryFrame.rearAxleSteeringAngle;

  double tanFrontSteeringAngle = std::tan(frontSteeringAngle);
  double tanRearSteeringAngle = std::tan(rearSteeringAngle);
  double instantaneousCurvature = (tanFrontSteeringAngle-tanRearSteeringAngle)/
    (frontWheelbase+rearWheelbase);

//  double frontInstantaneousCurvature =tanFrontSteeringAngle/frontWheelbase;
//  double frontInstantaneousCurvatureHalfTrack_ = frontInstantaneousCurvature*frontHalfTrack;
  double frontInstantaneousCurvatureHalfTrack_ = instantaneousCurvature*frontHalfWheelTrack;

  double frontAlphaLeft = 1 - frontInstantaneousCurvatureHalfTrack_;
  double frontAlphaRight = 1 + frontInstantaneousCurvatureHalfTrack_;
  double squareTanFrontSteeringAngle = tanFrontSteeringAngle*tanFrontSteeringAngle;
  double frontBetaLeft =  std::sqrt(frontAlphaLeft*frontAlphaLeft+squareTanFrontSteeringAngle);
  double frontBetaRight = std::sqrt(frontAlphaRight*frontAlphaRight+squareTanFrontSteeringAngle);
  double frontGammaLeft = frontBetaLeft - instantaneousCurvature*frontHubCarrierOffset;
  double frontGammaRight = frontBetaRight + instantaneousCurvature*frontHubCarrierOffset;

//  double rearInstantaneousCurvature = tanRearSteeringAngle/rearWheelbase;
//  double rearInstantaneousCurvatureHalfTrack_ = rearInstantaneousCurvature*rearHalfTrack;
  double rearInstantaneousCurvatureHalfTrack_ = instantaneousCurvature*rearHalfTrack;

  double rearAlphaLeft = 1 - rearInstantaneousCurvatureHalfTrack_;
  double rearAlphaRight = 1 + rearInstantaneousCurvatureHalfTrack_;
  double squareTanRearSteeringAngle = tanRearSteeringAngle*tanRearSteeringAngle;
  double rearBetaLeft =  std::sqrt(rearAlphaLeft*rearAlphaLeft+squareTanRearSteeringAngle);
  double rearBetaRight = std::sqrt(rearAlphaRight*rearAlphaRight+squareTanRearSteeringAngle);
  double rearGammaLeft = rearBetaLeft - instantaneousCurvature*rearHubCarrierOffset;
  double rearGammaRight = rearBetaRight + instantaneousCurvature*rearHubCarrierOffset;

  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(6, 6);
  covariance(0, 0) = wheelSpeedVariance;
  covariance(1, 1) = wheelSpeedVariance;
  covariance(2, 2) = wheelSpeedVariance;
  covariance(3, 3) = wheelSpeedVariance;
  covariance(4, 4) = steeringAngleVariance;
  covariance(5, 5) = steeringAngleVariance;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 6);
//  J(0,0) = 0.25/frontGammaLeft;
//  J(0,1) = 0.25/frontGammaRight;
//  J(0,2) = 0.25/rearGammaLeft;
//  J(0,3) = 0.25/rearGammaRight;
//  J(0,4) += ((frontAlphaLeft*frontHalfTrack/frontWheelbase+tanFrontSteeringAngle)/frontBetaLeft - frontHubCarrierOffset/frontWheelbase)/(frontGammaLeft*frontGammaLeft);
//  J(0,4) += ((frontAlphaRight*frontHalfTrack/frontWheelbase+tanFrontSteeringAngle)/frontBetaRight + frontHubCarrierOffset/frontWheelbase)/(frontGammaRight*frontGammaRight);
//  J(0,4) *= 0.25*(1+tanFrontSteeringAngle*tanFrontSteeringAngle);
//  J(0,5) += ((-rearAlphaLeft*rearHalfTrack/rearWheelbase+tanFrontSteeringAngle)/rearBetaLeft + rearHubCarrierOffset/rearWheelbase)/(rearGammaLeft*rearGammaLeft);
//  J(0,5) += ((-rearAlphaRight*rearHalfTrack/rearWheelbase+tanFrontSteeringAngle)/rearBetaRight - rearHubCarrierOffset/rearWheelbase)/(rearGammaRight*rearGammaRight);
//  J(0,5) *= 0.25*(1+tanFrontSteeringAngle*tanFrontSteeringAngle);
//  J(1,4) =1;
//  J(2,5) =1;


  twoAxleSteeringMeasure.frontSteeringAngle = frontSteeringAngle;
  twoAxleSteeringMeasure.rearSteeringAngle = rearSteeringAngle;

  twoAxleSteeringMeasure.longitudinalSpeed = 0.25*(frontLeftWheelSpeed/frontGammaLeft+
                                                   frontRightWheelSpeed/frontGammaRight+
                                                   rearLeftWheelSpeed/rearGammaLeft+
                                                   rearRightWheelSpeed/rearGammaRight);

  twoAxleSteeringMeasure.covariance =  J*covariance*J.transpose();
}

}
