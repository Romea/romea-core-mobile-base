//romea
#include "romea_core_mobile_base/kinematic/axle_steering/InverseTwoAxleSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>
#include <iostream>

namespace romea {


//-----------------------------------------------------------------------------
void inverseKinematic(const TwoAxleSteeringKinematic::Parameters & parameters,
                      const OdometryFrame2AS4WD & odometryFrame,
                      TwoAxleSteeringMeasure & twoAxleSteeringMeasure)
{

  const double &wheelSpeedVariance = parameters.wheelSpeedVariance;
  const double &steeringAngleVariance = parameters.steeringAngleVariance;

  const double frontWheelbase = parameters.frontWheelBase;
  const double frontHalfWheelTrack = parameters.frontWheelTrack/2;
  const double frontHubCarrierOffset = parameters.frontHubCarrierOffset;
  const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelSpeed;
  const double & frontRightWheelSpeed = odometryFrame.frontRightWheelSpeed;
  const double & frontSteeringAngle = odometryFrame.frontAxleSteeringAngle;

  const double rearWheelbase = parameters.rearWheelBase;
  const double rearHalfTrack = parameters.rearWheelTrack/2;
  const double rearHubCarrierOffset = parameters.rearHubCarrierOffset;
  const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelSpeed;
  const double & rearRightWheelSpeed = odometryFrame.rearRightWheelSpeed;
  const double & rearSteeringAngle = odometryFrame.rearAxleSteeringAngle;

  double tanFrontSteeringAngle = std::tan(frontSteeringAngle);
  double tanRearSteeringAngle = std::tan(rearSteeringAngle);
  double instantaneousCurvature = (tanFrontSteeringAngle-tanRearSteeringAngle)/(frontWheelbase+rearWheelbase);

//  double frontInstantaneousCurvature =tanFrontSteeringAngle/frontWheelbase;
//  double frontInstantaneousCurvatureHalfTrack_ = frontInstantaneousCurvature*frontHalfTrack;
  double frontInstantaneousCurvatureHalfTrack_ = instantaneousCurvature*frontHalfWheelTrack;

  double frontAlphaLeft = 1 - frontInstantaneousCurvatureHalfTrack_;
  double frontAlphaRight =1 + frontInstantaneousCurvatureHalfTrack_;
  double squareTanFrontSteeringAngle = tanFrontSteeringAngle*tanFrontSteeringAngle;
  double frontBetaLeft =  std::sqrt(frontAlphaLeft*frontAlphaLeft+squareTanFrontSteeringAngle);
  double frontBetaRight = std::sqrt(frontAlphaRight*frontAlphaRight+squareTanFrontSteeringAngle);
  double frontGammaLeft = frontBetaLeft - instantaneousCurvature*frontHubCarrierOffset;
  double frontGammaRight = frontBetaRight + instantaneousCurvature*frontHubCarrierOffset;



//  double rearInstantaneousCurvature = tanRearSteeringAngle/rearWheelbase;
//  double rearInstantaneousCurvatureHalfTrack_ = rearInstantaneousCurvature*rearHalfTrack;
  double rearInstantaneousCurvatureHalfTrack_ = instantaneousCurvature*rearHalfTrack;

  double rearAlphaLeft = 1 - rearInstantaneousCurvatureHalfTrack_;
  double rearAlphaRight =1 + rearInstantaneousCurvatureHalfTrack_;
  double squareTanRearSteeringAngle = tanRearSteeringAngle*tanRearSteeringAngle;
  double rearBetaLeft =  std::sqrt(rearAlphaLeft*rearAlphaLeft+squareTanRearSteeringAngle);
  double rearBetaRight = std::sqrt(rearAlphaRight*rearAlphaRight+squareTanRearSteeringAngle);
  double rearGammaLeft = rearBetaLeft - instantaneousCurvature*rearHubCarrierOffset;
  double rearGammaRight = rearBetaRight + instantaneousCurvature*rearHubCarrierOffset;

  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(6,6);
  covariance(0,0)=wheelSpeedVariance;
  covariance(1,1)=wheelSpeedVariance;
  covariance(2,2)=wheelSpeedVariance;
  covariance(3,3)=wheelSpeedVariance;
  covariance(4,4)=steeringAngleVariance;
  covariance(5,5)=steeringAngleVariance;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3,6);
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


  twoAxleSteeringMeasure.frontSteeringAngle= frontSteeringAngle;
  twoAxleSteeringMeasure.rearSteeringAngle= rearSteeringAngle;

  twoAxleSteeringMeasure.longitudinalSpeed = 0.25*(frontLeftWheelSpeed/frontGammaLeft+
                                                   frontRightWheelSpeed/frontGammaRight+
                                                   rearLeftWheelSpeed/rearGammaLeft+
                                                   rearRightWheelSpeed/rearGammaRight);

  twoAxleSteeringMeasure.covariance =  J*covariance*J.transpose();
}

////-----------------------------------------------------------------------------
//KinematicMeasure inverseKinematic2AS4WD(const TwoAxleSteeringKinematic & kinematic,
//                                        const OdometryFrame2AS4WD & odometryFrame,
//                                        const double &wheelSpeedVariance,
//                                        const double &steeringAngleVariance)
//{

//  //Compute instantaneous curvature and its variance

//  const double & wheelSpeedVariance = odometry.getWheelSpeedVariance();
//  const double & wheelAngleVariance = odometry.getWheelAngleVariance();
//  const double & frontWheelBase = kinematic.getFrontWheelBase();
//  const double & rearWheelBase = kinematic.getRearWheelBase();
//  const double & track = kinematic.getTrack();

//  double wheelBase= frontWheelBase+rearWheelBase;

//  //left information
//  const double & frontLeftWheelAngle = odometryFrame.getFrontLeftWheelAngle();
//  const double & frontLeftWheelSpeed = odometryFrame.getFrontLeftWheelSpeed();
//  const double & rearLeftWheelAngle = odometryFrame.getRearLeftWheelAngle();
//  const double & rearLeftWheelSpeed = odometryFrame.getRearLeftWheelSpeed();


//  double leftLinearSpeed = 0.5*(frontLeftWheelSpeed*std::cos(frontLeftWheelAngle)+
//                                rearLeftWheelSpeed*std::cos(rearLeftWheelAngle));

//  double tanLeftBeta = (rearWheelBase*std::tan(frontLeftWheelAngle)+
//                        frontWheelBase*std::tan(rearLeftWheelAngle))/wheelBase;

//  double leftLateralSpeed = std::abs(leftLinearSpeed)*tanLeftBeta;

//  double leftInstantaneousCurvature = (std::tan(frontLeftWheelAngle)-std::tan(rearLeftWheelAngle))/wheelBase;

//  double leftGamma=1+leftInstantaneousCurvature*track/2;

//  double leftAlpha = leftGamma*leftGamma + tanLeftBeta*tanLeftBeta;

//  double leftAngularSpeed = leftInstantaneousCurvature*leftLinearSpeed;

////  std::cout << " leftLinearSpeed " << leftLinearSpeed << std::endl;
////  std::cout << " tanLeftBeta " << tanLeftBeta << " "<< std::atan(tanLeftBeta)*180/M_PI<<std::endl;
////  std::cout <<  " leftGamma " << leftGamma << std::endl;

//  //right information
//  const double & frontRightWheelAngle = odometryFrame.getFrontRightWheelAngle();
//  const double & frontRightWheelSpeed = odometryFrame.getFrontRightWheelSpeed();
//  const double & rearRightWheelAngle = odometryFrame.getRearRightWheelAngle();
//  const double & rearRightWheelSpeed = odometryFrame.getRearRightWheelSpeed();

//  double rightLinearSpeed = 0.5*(frontRightWheelSpeed*std::cos(frontRightWheelAngle)+
//                                 rearRightWheelSpeed*std::cos(rearRightWheelAngle));

//  double tanRightBeta = (rearWheelBase*std::tan(frontRightWheelAngle)+
//                         frontWheelBase*std::tan(rearRightWheelAngle))/wheelBase;


//  double rightLateralSpeed = std::abs(rightLinearSpeed)*tanRightBeta;

//  double rightInstantaneousCurvature = (std::tan(frontRightWheelAngle)-std::tan(rearRightWheelAngle))/wheelBase;

//  double rightGamma=1-rightInstantaneousCurvature*track/2;

//  double rightAlpha = rightGamma*rightGamma + tanRightBeta*tanRightBeta;

//  double rightAngularSpeed = leftInstantaneousCurvature*rightLinearSpeed;

////  std::cout << " rightLinearSpeed " << rightLinearSpeed << std::endl;
////  std::cout << " tanRightBeta " << tanRightBeta << " "<< std::atan(tanRightBeta)*180/M_PI<<std::endl;
////  std::cout << " rightGamma " <<rightGamma << std::endl;


//  //kinematic value
//  double linearSpeed = 0.5*(leftLinearSpeed+rightLinearSpeed);

//  double lateralSpeed = 0.5*(leftLateralSpeed+rightLateralSpeed);

//  double instantaneousCurvature = 0.5*(leftInstantaneousCurvature / std::sqrt(leftAlpha) +rightInstantaneousCurvature / std::sqrt(rightAlpha));

//  double angularSpeed =  0.5*(rightAngularSpeed*leftAngularSpeed);

////  std::cout << "odometry " << std::endl;
////  std::cout <<"wheel angles "<< frontLeftWheelAngle*180/M_PI << " "<< frontRightWheelAngle*180/M_PI << " " << rearLeftWheelAngle*180/M_PI << " "<< rearRightWheelAngle*180/M_PI << std::endl;
////  std::cout <<"wheel speeds "<< frontLeftWheelSpeed << " "<< frontRightWheelSpeed << " " << rearLeftWheelSpeed << " "<< rearRightWheelSpeed << std::endl;

////  std::cout << "curvature" <<std::endl;
////  std::cout << instantaneousCurvature << std::endl;
////  std::cout << leftInstantaneousCurvature << " "<< rightInstantaneousCurvature<<std::endl;
////  std::cout << leftInstantaneousCurvature / std::sqrt(leftAlpha) <<" "<<
////               rightInstantaneousCurvature / std::sqrt(rightAlpha) <<" "<< 0.5*(leftInstantaneousCurvature / std::sqrt(leftAlpha) +rightInstantaneousCurvature / std::sqrt(rightAlpha))<<std::endl;


//  Eigen::Array44d Jws = Eigen::Array44d::Zero(4,4);
//  Eigen::Array44d Jwa = Eigen::Array44d::Zero(4,4);

//  Eigen::Array4d wheelBases(frontWheelBase,frontWheelBase,rearWheelBase,rearWheelBase);
//  Eigen::Array4d wheelAngles(frontLeftWheelAngle ,frontRightWheelAngle, rearLeftWheelAngle,rearRightWheelAngle);
//  Eigen::Array4d wheelSpeeds(frontLeftWheelSpeed ,frontRightWheelSpeed, rearLeftWheelSpeed,rearRightWheelSpeed);
//  Eigen::Array4d tanBetas(tanLeftBeta,tanRightBeta,tanLeftBeta,tanRightBeta);
//  Eigen::Array4d gammas(leftGamma, leftGamma, rightGamma, rightGamma);
//  Eigen::Array4d alphas = gammas.square() + tanBetas.square();
//  Eigen::Array4d cs(leftInstantaneousCurvature, rightInstantaneousCurvature,leftInstantaneousCurvature,rightInstantaneousCurvature);

//  Eigen::Array4d onePlusTanBetasSquare = Eigen::Array4d::Ones(4)+tanBetas.square();
//  Eigen::Array4d dTanBetasdWheelAngles = onePlusTanBetasSquare*wheelBases/wheelBase;
//  Eigen::Array4d dCsdWheelAngles = (Eigen::Array4d() << 1,-1,1,-1).finished()*onePlusTanBetasSquare;
//  Eigen::Array4d dAlphasdWheelAngles = gammas*dCsdWheelAngles*track + 2*tanBetas* dTanBetasdWheelAngles;



//  Jws.col(0) = std::sqrt(0.25)*wheelAngles.cos();
//  Jws.col(1) = Jws.col(0)*tanBetas;
//  Jws.col(3) = cs;

//  Jwa.col(0) = -std::sqrt(0.25)*wheelAngles.sin()*wheelSpeeds;
//  Jwa.col(1) = Jwa.col(0)*tanBetas + wheelSpeeds*dTanBetasdWheelAngles;
//  Jwa.col(2) = dCsdWheelAngles*alphas.pow(-0.5) + 0.5*cs*alphas.pow(-1.5)*dAlphasdWheelAngles;
//  Jwa.col(3) = Jwa.col(2)*wheelSpeeds;

//  Eigen::Matrix4d covariance = wheelSpeedVariance*Jws.transpose()*Jws + wheelAngleVariance*Jwa.transpose()*Jwa;


////  std::cout << " covariance " << std::endl;
////  std::cout <<  covariance  << std::endl;

//  return KinematicMeasure(odometryFrame.getDuration(),
//                          linearSpeed,
//                          lateralSpeed,
//                          angularSpeed,
//                          instantaneousCurvature,
//                          covariance,
//                          odometryFrame.parkingStation());



//}

}
