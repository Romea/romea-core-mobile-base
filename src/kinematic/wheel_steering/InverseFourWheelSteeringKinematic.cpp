//romea
#include "romea_core_mobile_base/kinematic/wheel_steering/InverseFourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp"
#include <romea_core_common/math/Algorithm.hpp>

#include <iostream>

namespace romea {


//-----------------------------------------------------------------------------
void inverseKinematic(const FourWheelSteeringKinematic::Parameters & parameters,
                      const OdometryFrame4WS4WD & odometryFrame,
                      TwoAxleSteeringMeasure & twoAxleSteeringMeasure)
{

  const double &wheelSpeedVariance = parameters.wheelSpeedVariance;
  const double &wheelAngleVariance = parameters.wheelAngleVariance;

  const double halfWheelTrack = parameters.wheelTrack/2;
  const double hubCarrierOffset = parameters.hubCarrierOffset;
  const double frontWheelBase = parameters.frontWheelBase;
  const double rearWheelBase = parameters.rearWheelBase;
  double wheelbase = frontWheelBase+ rearWheelBase;

  const double & frontLeftWheelAngle = odometryFrame.frontLeftWheelAngle;
  const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelSpeed;
  const double & rearLeftWheelAngle = odometryFrame.rearLeftWheelAngle;
  const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelSpeed;
  double frontTanLeft = std::tan(frontLeftWheelAngle);
  double frontCosLeft = std::cos(frontLeftWheelAngle);
  double frontSinLeft = std::cos(frontLeftWheelAngle);
  double rearTanLeft = std::tan(rearLeftWheelAngle);
  double rearCosLeft = std::cos(rearLeftWheelAngle);
  double rearSinLeft = std::sin(rearLeftWheelAngle);

  double KLeft = (frontTanLeft-rearTanLeft)/wheelbase;
  double alphaLeft = 1 + KLeft*halfWheelTrack;
  double frontBetaLeft = 1-hubCarrierOffset*KLeft*frontCosLeft;
  double rearBetaLeft = 1-hubCarrierOffset*KLeft*rearCosLeft;


  const double & frontRightWheelAngle = odometryFrame.frontRightWheelAngle;
  const double & frontRightWheelSpeed = odometryFrame.frontRightWheelSpeed;
  const double & rearRightWheelAngle = odometryFrame.rearRightWheelAngle;
  const double & rearRightWheelSpeed = odometryFrame.rearRightWheelSpeed;


  double frontTanRight = std::tan(frontRightWheelAngle);
  double frontCosRight = std::cos(frontRightWheelAngle);
  double frontSinRight = std::cos(frontRightWheelAngle);
  double rearTanRight = std::tan(rearRightWheelAngle);
  double rearCosRight = std::cos(rearRightWheelAngle);
  double rearSinRight = std::cos(rearRightWheelAngle);

  double KRight = (frontTanRight-rearTanRight)/wheelbase;
  double alphaRight = 1 -KRight*halfWheelTrack;
  double frontBetaRight = 1+hubCarrierOffset*KRight*frontCosRight;
  double rearBetaRight =  1+hubCarrierOffset*KRight*rearCosRight;


  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(8,8);
  covariance(0,0)=wheelSpeedVariance;
  covariance(1,1)=wheelSpeedVariance;
  covariance(2,2)=wheelSpeedVariance;
  covariance(3,3)=wheelSpeedVariance;
  covariance(4,4)=wheelAngleVariance;
  covariance(5,5)=wheelAngleVariance;
  covariance(6,6)=wheelAngleVariance;
  covariance(7,7)=wheelAngleVariance;

//  std::cout <<"covariance"<< std::endl;
//  std::cout <<covariance<< std::endl;


  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3,8);
  J(0,0) = 0.25*frontCosLeft*alphaLeft/frontBetaLeft;
  J(0,1) = 0.25*rearCosLeft*alphaLeft/rearBetaLeft;
  J(0,2) = 0.25*frontCosRight*alphaRight/frontBetaRight;
  J(0,3) = 0.25*rearCosRight*alphaRight/rearBetaRight;

//  double frontGammaLeftFront = (frontCosLeft+frontSinLeft*rearTanLeft)/wheelbase;
//  double rearGammaLeftFront =   rearCosLeft*(1+rearTanLeft*rearTanLeft)/wheelbase;

//  J(0,4)+= (-frontSinLeft + frontGammaLeftFront*halfTrack)/ frontBetaLeft;
//  J(0,4)+= hubCarrierOffset*frontGammaLeftFront*alphaLeft / frontBetaLeft*frontBetaLeft;
//  J(0,4)+=  rearGammaLeftFront / frontBetaLeft;

//  double frontGammaLeftRear = - frontCosLeft*(1+rearTanLeft*rearTanLeft)/wheelbase;
//  double rearGammaLeftRear = (-rearSinLeft*frontTanLeft+rearCosLeft)/wheelbase;

//  J(0,5)+= (-rearSinLeft + frontGammaLeftRear*halfTrack)/rearBetaLeft;
//  J(0,5)-= hubCarrierOffset*frontGammaLeftRear*alphaLeft/rearBetaLeft*rearBetaLeft;
//  J(0,5)+=  rearGammaLeftRear / rearBetaLeft;


//  double frontGammaRightFront = (frontCosLeft+frontSinLeft*rearTanLeft)/wheelbase;
//  double rearGammaRightFront =   rearCosLeft*(1+rearTanLeft*rearTanLeft)/wheelbase;

//  J(0,4)+= (-frontSinLeft + frontGammaLeftFront*halfTrack)/ frontBetaLeft;
//  J(0,4)+= hubCarrierOffset*frontGammaLeftFront*alphaLeft / frontBetaLeft*frontBetaLeft;
//  J(0,4)+=  rearGammaLeftFront / frontBetaLeft;

//  double frontGammaRightRear = - frontCosLeft*(1+rearTanLeft*rearTanLeft)/wheelbase;
//  double rearGammaRigthRear = (-rearSinLeft*frontTanLeft+rearCosLeft)/wheelbase;

//  J(0,5)+= (-rearSinLeft + frontGammaLeftRear*halfTrack)/rearBetaLeft;
//  J(0,5)-= hubCarrierOffset*frontGammaLeftRear*alphaLeft/rearBetaLeft*rearBetaLeft;
//  J(0,5)+=  rearGammaLeftRear / rearBetaLeft;


  double tanFrontSteeringAngle=0.5*(frontTanLeft/alphaLeft + frontTanRight/alphaRight);
  J(1,4) =  0.5*(1+frontTanLeft*frontTanLeft)  *(alphaLeft -frontTanLeft*halfWheelTrack/wheelbase)/(alphaLeft*alphaLeft);
  J(1,5) =  0.5*(1+rearTanLeft*rearTanLeft)    *(           frontTanLeft*halfWheelTrack/wheelbase)/(alphaLeft*alphaLeft);
  J(1,6) =  0.5*(1+frontTanRight*frontTanRight)*(alphaRight+frontTanRight*halfWheelTrack/wheelbase)/(alphaRight*alphaRight);
  J(1,7) =  0.5*(1+rearTanRight*rearTanRight)  *(          -frontTanRight*halfWheelTrack/wheelbase)/(alphaRight*alphaRight);
  J.row(1)*=1/(1+tanFrontSteeringAngle*tanFrontSteeringAngle);


  double tanRearSteeringAngle=0.5*(rearTanLeft/alphaLeft + rearTanRight/alphaRight);
  J(2,4) =  0.5*(1+frontTanLeft*frontTanLeft)  *(          -rearTanLeft*halfWheelTrack/wheelbase)/(alphaLeft*alphaLeft);
  J(2,5) =  0.5*(1+rearTanLeft*rearTanLeft)    *(alphaLeft +rearTanLeft*halfWheelTrack/wheelbase)/(alphaLeft*alphaLeft);
  J(2,6) =  0.5*(1+frontTanRight*frontTanRight)*(           rearTanRight*halfWheelTrack/wheelbase)/(alphaRight*alphaRight);
  J(2,7) =  0.5*(1+rearTanRight*rearTanRight)  *(alphaRight-rearTanRight*halfWheelTrack/wheelbase)/(alphaRight*alphaRight);
  J.row(2)*=1/(1+tanRearSteeringAngle*tanFrontSteeringAngle);

//  std::cout << " intermediare" << std::endl;
//  std::cout << frontLeftWheelSpeed <<" "<<frontLeftWheelSpeed*frontCosLeft <<" "<<alphaLeft <<" "<< frontBetaLeft<< std::endl;
//  std::cout << rearLeftWheelSpeed <<" "<<rearLeftWheelSpeed*rearCosLeft <<" "<<alphaLeft <<" "<< frontBetaLeft<< std::endl;
//  std::cout << frontRightWheelSpeed <<" "<<frontRightWheelSpeed*frontCosRight <<" "<<alphaRight <<" "<< frontBetaRight<< std::endl;
//  std::cout << rearRightWheelSpeed <<" "<<rearRightWheelSpeed*rearCosRight <<" "<<alphaRight <<" "<< rearBetaRight<< std::endl;

  twoAxleSteeringMeasure.longitudinalSpeed = 0.25*(frontLeftWheelSpeed*frontCosLeft*alphaLeft/frontBetaLeft+
                                                   rearLeftWheelSpeed*rearCosLeft*alphaLeft/rearBetaLeft+
                                                   frontRightWheelSpeed*frontCosRight*alphaRight/frontBetaRight+
                                                   rearRightWheelSpeed*rearCosRight*alphaRight/rearBetaRight);

  twoAxleSteeringMeasure.frontSteeringAngle= std::atan(0.5*(frontTanLeft/alphaLeft + frontTanRight/alphaRight));
  twoAxleSteeringMeasure.rearSteeringAngle= std::atan(0.5*(rearTanLeft/alphaLeft + rearTanRight/alphaRight));


//  std::cout << " inverse kinematic" << std::endl;
//  std::cout << twoAxleSteeringMeasure.longitudinalSpeed <<" "
//            <<  twoAxleSteeringMeasure.frontSteeringAngle<<" "
//             <<  twoAxleSteeringMeasure.rearSteeringAngle<< std::endl;

  twoAxleSteeringMeasure.covariance =  J*covariance*J.transpose();

}


////-----------------------------------------------------------------------------
//KinematicMeasure inverseKinematic4WS4WD(const FourWheelSteeringKinematic & kinematic,
//                                        const OdometryFrame4WS4WD & odometryFrame,
//                                        const double & wheelSpeedVariance,
//                                        const double & wheelAngleVariance)
//{

//  //Compute instantaneous curvature and its variance
//  const double frontHubCarrierOffset = kinematic.getHubCarrierOffset("front_hub_carrier_offset");
//  const double rearHubCarrierOffset = kinematic.getHubCarrierOffset("rear_hub_carrier_offset");
//  const double frontWheelBase = kinematic.getWheelBase("front_wheelbase");
//  const double rearWheelBase = kinematic.getWheelBase("rear_wheelbase");
//  const double frontTrack = kinematic.getTrack("front_track");
//  const double rearTrack = kinematic.getTrack("rear_track");
//  const double wheelBase= frontWheelBase+rearWheelBase;
//  const double track = (frontTrack+rearTrack)/2.;
//  const double halfTrack = track/2.;

//  //left information
//  const double & frontLeftWheelAngle = odometryFrame.frontLeftWheelAngle;
//  const double & frontLeftWheelSpeed = odometryFrame.frontLeftWheelSpeed;
//  const double & rearLeftWheelAngle = odometryFrame.rearLeftWheelAngle;
//  const double & rearLeftWheelSpeed = odometryFrame.rearLeftWheelSpeed;


//  double leftInstantaneousCurvature = (std::tan(frontLeftWheelAngle)-std::tan(rearLeftWheelAngle))/wheelBase;

//  double leftLinearSpeed = 0.5*(frontLeftWheelSpeed*std::cos(frontLeftWheelAngle)/
//                                (1-frontHubCarrierOffset*leftInstantaneousCurvature*std::cos(frontLeftWheelAngle))+
//                                rearLeftWheelSpeed*std::cos(rearLeftWheelAngle)/
//                                (1-rearHubCarrierOffset*leftInstantaneousCurvature*std::cos(rearLeftWheelAngle)));

//  double tanLeftBeta = (rearWheelBase*std::tan(frontLeftWheelAngle)+
//                        frontWheelBase*std::tan(rearLeftWheelAngle))/wheelBase;


//  //right information
//  const double & frontRightWheelAngle = odometryFrame.frontRightWheelAngle;
//  const double & frontRightWheelSpeed = odometryFrame.frontRightWheelSpeed;
//  const double & rearRightWheelAngle = odometryFrame.rearRightWheelAngle;
//  const double & rearRightWheelSpeed = odometryFrame.rearRightWheelSpeed;

//  double rightInstantaneousCurvature = (std::tan(frontRightWheelAngle)-std::tan(rearRightWheelAngle))/wheelBase;

//  double rightLinearSpeed = 0.5*(frontRightWheelSpeed*std::cos(frontRightWheelAngle)/
//                                 (1+frontHubCarrierOffset*rightInstantaneousCurvature*std::cos(frontRightWheelAngle))+
//                                 rearRightWheelSpeed*std::cos(rearRightWheelAngle)/
//                                 (1+rearHubCarrierOffset*rightInstantaneousCurvature*std::cos(rearRightWheelAngle)));

//  double tanRightBeta = (rearWheelBase*std::tan(frontRightWheelAngle)+
//                         frontWheelBase*std::tan(rearRightWheelAngle))/wheelBase;


//  //kinematic value
//  double linearSpeed = 0.5*(leftLinearSpeed+rightLinearSpeed);

//  double instantaneousCurvature= TwoWheelSteeringKinematic::computeInstantaneousCurvature(leftInstantaneousCurvature,rightInstantaneousCurvature,track);

//  double tanBeta= 0.5*(tanLeftBeta/(1+leftInstantaneousCurvature*track/2)+
//                       tanRightBeta/(1-rightInstantaneousCurvature*track/2));

//  double beta = std::atan(tanBeta);


//  Eigen::Array4d wheelBases(frontWheelBase,frontWheelBase,rearWheelBase,rearWheelBase);
//  Eigen::Array4d wheelAngles(frontLeftWheelAngle ,frontRightWheelAngle, rearLeftWheelAngle,rearRightWheelAngle);
//  Eigen::Array4d wheelSpeeds(frontLeftWheelSpeed ,frontRightWheelSpeed, rearLeftWheelSpeed,rearRightWheelSpeed);
//  Eigen::Array4d cs(leftInstantaneousCurvature, rightInstantaneousCurvature,leftInstantaneousCurvature,rightInstantaneousCurvature);

//  Eigen::Array4d alphas(1 + std::pow(std::tan(frontLeftWheelAngle),2)/wheelBase,
//                        1 + std::pow(std::tan(frontRightWheelAngle),2)/wheelBase,
//                        1 + std::pow(std::tan(rearLeftWheelAngle),2)/wheelBase,
//                        1 + std::pow(std::tan(rearRightWheelAngle),2)/wheelBase);

//  Eigen::Array4d betas(2*std::pow(1+leftInstantaneousCurvature*halfTrack,2),
//                       2*std::pow(1+leftInstantaneousCurvature*halfTrack,2),
//                       2*std::pow(1-rightInstantaneousCurvature*halfTrack,2),
//                       2*std::pow(1-rightInstantaneousCurvature*halfTrack,2));

//  Eigen::Array4d theta(1 + leftInstantaneousCurvature*halfTrack  - std::tan(frontLeftWheelAngle),
//                       1 + leftInstantaneousCurvature*halfTrack  + std::tan(rearLeftWheelAngle),
//                       1 - rightInstantaneousCurvature*halfTrack  - std::tan(frontRightWheelAngle),
//                       1 - rightInstantaneousCurvature*halfTrack  + std::tan(rearRightWheelAngle));

//  double gamma = 1+ tanBeta*tanBeta;

//  Eigen::MatrixXd Jws = Eigen::MatrixXd::Zero(4,3);
//  Jws.col(0) = std::sqrt(0.25)*wheelAngles.cos();
//  Jws.col(2) = cs;

//  Eigen::MatrixXd Jwa = Eigen::MatrixXd::Zero(4,3);
//  Jwa.col(0) = -std::sqrt(0.25)*wheelAngles.sin()*wheelSpeeds;
//  Jwa.col(1) = alphas*wheelBases;
//  Jwa.col(2) = alphas*theta/(betas*gamma);

//  Eigen::Matrix3d covariance = wheelSpeedVariance*Jws.transpose()*Jws + wheelAngleVariance*Jwa.transpose()*Jwa;

//  Eigen::Matrix3d Jbeta = Eigen::Matrix3d::Identity();
//  Jbeta(0,0)=1/std::cos(beta);
//  Jbeta(0,1)=Jbeta(1,0)=linearSpeed*std::tan(beta)/std::cos(beta);
//  Jbeta(1,2)=Jbeta(2,1)= -std::sin(beta);

//  Eigen::MatrixXd Jw = Eigen::MatrixXd::Zero(4,3);
//  Jw(0,0)=1;
//  Jw(1,1)=1;
//  Jw(2,0)=instantaneousCurvature*std::cos(beta);
//  Jw(2,2)= linearSpeed/std::cos(beta);
//  Jw(3,2)=1;


//  KinematicMeasure kinematicMeasure;
//  kinematicMeasure.beta=beta;
//  kinematicMeasure.speed = linearSpeed/std::cos(beta);
//  kinematicMeasure.instantaneousCurvature = instantaneousCurvature*std::cos(beta);
//  kinematicMeasure.angularSpeed=kinematicMeasure.instantaneousCurvature* kinematicMeasure.speed ;
//  kinematicMeasure.covariance = Jw*Jbeta*covariance*Jbeta.transpose()*Jw.transpose();

//  return kinematicMeasure;


//}

}
