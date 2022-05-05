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

}
