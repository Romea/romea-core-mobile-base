//romea
#include "romea_odo/kinematic/axle_steering/InverseOneAxleSteeringKinematic.hpp"

namespace romea {


//-----------------------------------------------------------------------------
void inverseKinematic(const OneAxleSteeringKinematic::Parameters & parameters,
                      const OdometryFrame1FAS2FWD & odometryFrame,
                      OneAxleSteeringMeasure & oneAxleSteeringMeasure)
{

  const double &wheelSpeedVariance = parameters.wheelSpeedVariance;
  const double &steeringAngleVariance = parameters.steeringAngleVariance;

  const double halfTrack = parameters.frontTrack/2;
  const double wheelBase = parameters.frontWheelBase+parameters.rearWheelBase;
  const double hubCarrierOffset = parameters.rearHubCarrierOffset;

  const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelSpeed;
  const double & frontRightWheelSpeed = odometryFrame.frontRightWheelSpeed;
  const double & frontSteeringAngle = odometryFrame.frontAxleSteeringAngle;

  double tanFrontSteeringAngle = std::tan(frontSteeringAngle);
  double instantaneousCurvature =tanFrontSteeringAngle/wheelBase;
  double instantaneousCurvatureHalfTrack_ = instantaneousCurvature*halfTrack;
  
  double alphaLeft = 1 - instantaneousCurvatureHalfTrack_;
  double alphaRight =1 + instantaneousCurvatureHalfTrack_;
  double squareTanFrontSteeringAngle = tanFrontSteeringAngle*tanFrontSteeringAngle;
  double betaLeft =  std::sqrt(alphaLeft*alphaLeft+squareTanFrontSteeringAngle);
  double betaRight = std::sqrt(alphaRight*alphaRight+squareTanFrontSteeringAngle);
  double gammaLeft = betaLeft - tanFrontSteeringAngle*hubCarrierOffset/wheelBase;
  double gammaRight = betaRight + tanFrontSteeringAngle*hubCarrierOffset/wheelBase;


  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0,0)=wheelSpeedVariance;
  covariance(1,1)=wheelSpeedVariance;
  covariance(2,2)=steeringAngleVariance;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2,3);
  J(0,0) = 0.5/gammaLeft;
  J(0,1) = 0.5/gammaRight;
  J(0,2) += (2*(alphaLeft*halfTrack/wheelBase+tanFrontSteeringAngle)/betaLeft - hubCarrierOffset/wheelBase)/(gammaLeft*gammaLeft);
  J(0,2) += (2*(alphaRight*halfTrack/wheelBase+tanFrontSteeringAngle)/betaRight - hubCarrierOffset/wheelBase)/(gammaRight*gammaRight);
  J(0,2) *= 0.5*(1+tanFrontSteeringAngle*tanFrontSteeringAngle);
  J(1,2) =1;
  
  oneAxleSteeringMeasure.steeringAngle= frontSteeringAngle;
  oneAxleSteeringMeasure.longitudinalSpeed = 0.5*(frontLeftWheelSpeed/gammaLeft +frontRightWheelSpeed/gammaRight);
  oneAxleSteeringMeasure.covariance =  J*covariance*J.transpose();
  
}

//-----------------------------------------------------------------------------
void inverseKinematic(const OneAxleSteeringKinematic::Parameters & parameters,
                      const OdometryFrame1FAS2RWD & odometryFrame,
                      OneAxleSteeringMeasure & oneAxleSteeringMeasure)
{
  
  const double &wheelSpeedVariance = parameters.wheelSpeedVariance;
  const double &steeringAngleVariance = parameters.steeringAngleVariance;

  const double & steeringAngle = odometryFrame.frontAxleSteeringAngle;
  const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelSpeed;
  const double & rearRightWheelSpeed = odometryFrame.rearRightWheelSpeed;
  
  oneAxleSteeringMeasure.steeringAngle= steeringAngle;
  oneAxleSteeringMeasure.longitudinalSpeed = 0.5*(rearLeftWheelSpeed+rearRightWheelSpeed);
  oneAxleSteeringMeasure.covariance(0,0)=  0.5*wheelSpeedVariance;
  oneAxleSteeringMeasure.covariance(1,1)=  steeringAngleVariance;
  
}

////-----------------------------------------------------------------------------
//KinematicMeasure inverseKinematic1FAS2FWD(const OneAxleSteeringKinematic & kinematic,
//                                          const OdometryFrame1FAS2FWD & odometryFrame,
//                                          const double & wheelSpeedVariance,
//                                          const double & steeringAngleVariance)
//{

//  const double track = kinematic.getTrack("front_track");
//  const double wheelBase = kinematic.getWheelBase("wheelbase");

//  const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelSpeed;
//  const double & frontRightWheelSpeed = odometryFrame.frontRightWheelSpeed;
//  const double & frontSteeringAngle = odometryFrame.frontAxleSteeringAngle;


//  //Compute kinematic values
//  double tanFrontSteeringAngle = std::tan(frontSteeringAngle);
//  double instantaneousCurvature =tanFrontSteeringAngle/wheelBase;
//  double instantaneousCurvatureHalfTrack_ = instantaneousCurvature*track/2.;

//  double alphaLeft = 1 - instantaneousCurvatureHalfTrack_;
//  double alphaRight =1 + instantaneousCurvatureHalfTrack_;
//  double squareTanFrontSteeringAngle = tanFrontSteeringAngle*tanFrontSteeringAngle;
//  double squareBetaLeft = 1/(alphaLeft*alphaLeft+squareTanFrontSteeringAngle);
//  double squareBetaRight = 1/(alphaRight*alphaRight+squareTanFrontSteeringAngle);
//  double betaLeft = std::sqrt(squareBetaLeft);
//  double betaRight = std::sqrt(squareBetaRight);
//  double linearSpeed = 0.5*(frontLeftWheelSpeed*betaLeft +frontRightWheelSpeed*betaRight);
//  double angularSpeed =instantaneousCurvature*linearSpeed ;


//  //compute covariance matrix
//  double gamma = (1 + squareTanFrontSteeringAngle)/wheelBase;
//  double squareBeta = squareBetaLeft +squareBetaRight;
//  double deltaLeft= 0.5*(-(alphaLeft)*track/2 + 2*tanFrontSteeringAngle*wheelBase)*gamma/(std::pow(squareBetaLeft,1.5));
//  double deltaRight= 0.5*((alphaRight)*track/2 + 2*tanFrontSteeringAngle*wheelBase)*gamma/(std::pow(squareBetaRight,1.5));
//  double delta = deltaLeft + deltaRight;

//  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
//  covariance(0,0)= 0.25*squareBeta*wheelSpeedVariance +delta*delta*steeringAngleVariance;
//  covariance(0,2)= delta*gamma*steeringAngleVariance;
//  covariance(2,0)= covariance(0,2);
//  covariance(2,2)= gamma*gamma*steeringAngleVariance;

//  Eigen::MatrixXd Jw = Eigen::MatrixXd::Zero(4,3);
//  Jw(0,0)=1;
//  Jw(1,1)=1;
//  Jw(2,0)=instantaneousCurvature;
//  Jw(2,2)=linearSpeed;
//  Jw(3,2)=1;

//  //Create kinematic frame
//  KinematicMeasure kinematicMeasure;
//  kinematicMeasure.speed=linearSpeed;
//  kinematicMeasure.beta=0.;
//  kinematicMeasure.angularSpeed=angularSpeed;
//  kinematicMeasure.instantaneousCurvature=instantaneousCurvature;
//  kinematicMeasure.covariance =Jw*covariance*Jw.transpose();
//  return kinematicMeasure;


//}

////-----------------------------------------------------------------------------
//KinematicMeasure inverseKinematic1FAS2RWD(const OneAxleSteeringKinematic & kinematic,
//                                          const OdometryFrame1FAS2RWD & odometryFrame,
//                                          const double &wheelSpeedVariance,
//                                          const double &steeringAngleVariance)
//{

//  const double wheelBase = kinematic.getWheelBase("wheelbase");
//  const double & steeringAngle = odometryFrame.frontAxleSteeringAngle;
//  const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelSpeed;
//  const double & rearRightWheelSpeed = odometryFrame.rearRightWheelSpeed;

//  //Compute kinematic values
//  double linearSpeed = 0.5*(rearLeftWheelSpeed+rearRightWheelSpeed);
//  double instantaneousCurvature = OneAxleSteeringKinematic::computeInstantaneousCurvature(steeringAngle,wheelBase);
//  double angularSpeed = instantaneousCurvature * linearSpeed;


//  //Compute covariance matrix
//  double alpha = (1 + std::pow(instantaneousCurvature*wheelBase,2))/wheelBase;
//  double squareAlpha = alpha*alpha;

//  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
//  covariance(0,0)=0.5*wheelSpeedVariance;
//  covariance(2,2)=squareAlpha*steeringAngleVariance;

//  Eigen::MatrixXd Jw = Eigen::MatrixXd::Zero(4,3);
//  Jw(0,0)=1;
//  Jw(1,1)=1;
//  Jw(2,0)=instantaneousCurvature;
//  Jw(2,2)=linearSpeed;
//  Jw(3,2)=1;

//  //Create kinematic frame
//  KinematicMeasure kinematicMeasure;
//  kinematicMeasure.speed=linearSpeed;
//  kinematicMeasure.beta=0.;
//  kinematicMeasure.angularSpeed=angularSpeed;
//  kinematicMeasure.instantaneousCurvature=instantaneousCurvature;
//  kinematicMeasure.covariance =Jw*covariance*Jw.transpose();
//  return kinematicMeasure;

//}

}

